#include "stm32f4xx_hal.h"
#include <array>
#include <stdexcept>
#include <unordered_map>

// Fidelity: RM0090 rev22 §§8.4.1–10 (GPIO), §9.2 (EXTICR), §12.3.6
// (PR W1C). Observe word operations only. No physical addresses, CPU, NVIC,
// electrical pins, bus timing, internal EXTI lines or SWIER generation.
// Port clocks are an explicit fixture precondition; SYSCFG clock requests are
// recorded. Register layouts are checked against ST types in registers.h.
// ES0182 §2.2.10: PH10/PI10 routing is rejected as unmodelled, not simulated.
// Confidence: bounded software model with direct controls; no silicon proof.
namespace p12 {
namespace {
enum class Kind { Config, Input, Output, SetReset, Lock, Route, Exti, Pending, Unsupported };
struct Description { Kind kind; unsigned port; unsigned width; unsigned startPin; std::string name; };
struct LockState { unsigned phase{}; std::uint16_t pins{}; bool locked{}; };
std::array<GPIO_TypeDef, 9> ports;
EXTI_TypeDef external;
SYSCFG_TypeDef routing;
std::array<LockState, 9> locks;
std::unordered_map<std::uintptr_t, Description> descriptions;
std::vector<Access> accesses;
std::vector<Callback> delivered;
bool clockEnabled{};

auto key(const volatile Register *reg) { return reinterpret_cast<std::uintptr_t>(reg); }
const Description &describe(const volatile Register *reg) {
    const auto found = descriptions.find(key(reg));
    if (found == descriptions.end()) throw std::logic_error("unregistered P12 register");
    return found->second;
}
void bind(Register &reg, Kind kind, unsigned port, unsigned width, unsigned startPin, const std::string &name) {
    reg.storage = 0;
    descriptions.emplace(key(&reg), Description{kind, port, width, startPin, name});
}
void requireSupported(const Description &d) {
    if (d.kind == Kind::Unsupported) throw std::logic_error("unmodelled P12 register: " + d.name);
    if (d.kind == Kind::Route && !clockEnabled) throw std::logic_error("SYSCFG accessed with clock disabled");
}
}

void reset() {
    descriptions.clear(); accesses.clear(); delivered.clear(); clockEnabled = false;
    locks = {};
    for (unsigned p = 0; p < ports.size(); ++p) {
        auto &g = ports[p];
        const auto name = "GPIO" + std::to_string(p) + ".";
        bind(g.MODER, Kind::Config, p, 2, 0, name + "MODER");
        bind(g.OTYPER, Kind::Config, p, 1, 0, name + "OTYPER");
        bind(g.OSPEEDR, Kind::Config, p, 2, 0, name + "OSPEEDR");
        bind(g.PUPDR, Kind::Config, p, 2, 0, name + "PUPDR");
        bind(g.IDR, Kind::Input, p, 1, 0, name + "IDR");
        bind(g.ODR, Kind::Output, p, 1, 0, name + "ODR");
        bind(g.BSRR, Kind::SetReset, p, 1, 0, name + "BSRR");
        bind(g.LCKR, Kind::Lock, p, 1, 0, name + "LCKR");
        bind(g.AFR[0], Kind::Config, p, 4, 0, name + "AFRL");
        bind(g.AFR[1], Kind::Config, p, 4, 8, name + "AFRH");
        // RM0090 §§8.4.1, 8.4.3, 8.4.4: port A/B debug defaults.
        if (p == 0) { g.MODER.storage = 0xa8000000U; g.OSPEEDR.storage = 0x0c000000U; g.PUPDR.storage = 0x64000000U; }
        if (p == 1) { g.MODER.storage = 0x00000280U; g.OSPEEDR.storage = 0x000000c0U; g.PUPDR.storage = 0x00000100U; }
    }
    bind(external.IMR, Kind::Exti, 0, 1, 0, "EXTI.IMR");
    bind(external.EMR, Kind::Exti, 0, 1, 0, "EXTI.EMR");
    bind(external.RTSR, Kind::Exti, 0, 1, 0, "EXTI.RTSR");
    bind(external.FTSR, Kind::Exti, 0, 1, 0, "EXTI.FTSR");
    bind(external.SWIER, Kind::Unsupported, 0, 1, 0, "EXTI.SWIER");
    bind(external.PR, Kind::Pending, 0, 1, 0, "EXTI.PR");
    bind(routing.MEMRMP, Kind::Unsupported, 0, 1, 0, "SYSCFG.MEMRMP");
    bind(routing.PMC, Kind::Unsupported, 0, 1, 0, "SYSCFG.PMC");
    bind(routing.CMPCR, Kind::Unsupported, 0, 1, 0, "SYSCFG.CMPCR");
    for (unsigned i = 0; i < 2; ++i) bind(routing.RESERVED[i], Kind::Unsupported, 0, 1, 0, "SYSCFG.RESERVED");
    for (unsigned i = 0; i < 4; ++i) bind(routing.EXTICR[i], Kind::Route, 0, 4, i * 4, "SYSCFG.EXTICR" + std::to_string(i + 1));
}

GPIO_TypeDef *gpio(unsigned port) {
    if (port >= ports.size()) throw std::out_of_range("P12 GPIO port");
    return &ports[port];
}
EXTI_TypeDef *exti() { return &external; }
SYSCFG_TypeDef *syscfg() { return &routing; }
void enableSyscfgClock() { clockEnabled = true; accesses.push_back({"SYSCFG.CLOCK", true, 1}); }
bool syscfgClockEnabled() { return clockEnabled; }
void seed(Register &reg, std::uint32_t value) { (void)describe(&reg); reg.storage = value; }
std::uint32_t peek(const Register &reg) { (void)describe(&reg); return reg.storage; }
void injectPending(std::uint16_t mask) { external.PR.storage |= mask; }
const std::vector<Access> &trace() { return accesses; }
const std::vector<Callback> &callbacks() { return delivered; }
void recordCallback(std::uint16_t pin) { delivered.push_back({pin, external.PR.storage}); }

std::uint32_t read(const volatile Register *reg) {
    const auto &d = describe(reg); requireSupported(d);
    std::uint32_t result = reg->storage;
    if (d.kind == Kind::SetReset) result = 0;
    if (d.kind == Kind::Lock) {
        auto &state = locks[d.port];
        if (state.phase == 3) { state.locked = true; state.phase = 4; }
        result = state.pins | (state.locked ? GPIO_LCKR_LCKK : 0U);
    }
    accesses.push_back({d.name, false, result});
    return result;
}

void write(volatile Register *reg, std::uint32_t value) {
    const auto &d = describe(reg); requireSupported(d);
    accesses.push_back({d.name, true, value});
    if (d.kind == Kind::Input) throw std::logic_error("write to GPIO input register");
    if (d.kind == Kind::Config && d.name.ends_with("PUPDR")) {
        for (unsigned pin = 0; pin < 16; ++pin)
            if (((value >> (2 * pin)) & 3U) == 3U)
                throw std::logic_error("reserved GPIO pull encoding");
    }
    if ((d.kind == Kind::Output || d.kind == Kind::Pending || d.kind == Kind::Exti ||
         d.kind == Kind::Route || (d.kind == Kind::Config && d.width == 1)) && (value & 0xffff0000U))
        throw std::logic_error("P12 reserved or unsupported register bits");
    if (d.kind == Kind::Route) {
        for (unsigned i = 0; i < 4; ++i) {
            const auto port = (value >> (i * 4)) & 15U;
            if (port > 8 || (d.startPin + i == 10 && port >= 7))
                throw std::logic_error("P12 invalid or erratum-dependent route");
        }
    }
    if (d.kind == Kind::SetReset) {
        auto &odr = ports[d.port].ODR.storage;
        odr &= ~(value >> 16U);
        odr |= value & 0xffffU; // RM0090: set wins simultaneous BS/BR.
        return;
    }
    if (d.kind == Kind::Pending) { reg->storage = reg->storage & ~value; return; }
    if (d.kind == Kind::Lock) {
        if (value & 0xfffe0000U) throw std::logic_error("reserved lock bits");
        auto &s = locks[d.port];
        if (s.locked) return;
        const bool keyBit = (value & GPIO_LCKR_LCKK) != 0;
        const auto pins = static_cast<std::uint16_t>(value);
        if (s.phase == 0 && keyBit) { s.pins = pins; s.phase = 1; }
        else if (s.phase == 1 && !keyBit && pins == s.pins) s.phase = 2;
        else if (s.phase == 2 && keyBit && pins == s.pins) s.phase = 3;
        else { s.phase = 0; s.pins = pins; }
        return;
    }
    if (d.kind == Kind::Config && locks[d.port].locked) {
        std::uint32_t protectedFields = 0;
        const auto pins = locks[d.port].pins;
        for (unsigned i = 0; i < 32 / d.width; ++i) {
            if (i + d.startPin < 16 && (pins & (1U << (i + d.startPin))))
                protectedFields |= ((1U << d.width) - 1U) << (i * d.width);
        }
        value = (value & ~protectedFields) | (reg->storage & protectedFields);
    }
    reg->storage = value;
}
} // namespace p12

void p12_parameter_failure(const char *expression, unsigned) { throw std::invalid_argument(expression); }

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    // Normal public weak callback override: observe pending state at delivery.
    p12::recordCallback(pin);
}
