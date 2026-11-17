// P12 review witnesses, SPI (Codex review of P12, P12C-T4A-001).
#include "SpiTestSupport.h"

TEST_CASE_FIXTURE(SpiFixture, "RV7 SPI in bidirectional transmit mode receives no frames, as master and as slave (P12C-T4A-001)") {
	// RM0090 28.3.4: in bidirectional mode, when transmitting (BIDIMODE = 1 and BIDIOE = 1) "No data are received": neither RXNE nor OVR is set.
	bool slave = false;
	SUBCASE("master") { slave = false; }
	SUBCASE("slave") { slave = true; }
	SPI1->CR1 = 0xC340u | (slave ? 0u : 4u);  // BIDIMODE, BIDIOE, SPE, SSM, SSI (+ MSTR for the master)
	for (unsigned k = 0; k < 2; ++k) {
		SPI1->DR = 0x5Au + k;
		if (slave) {
			hostSpiSlaveFrame(0, 0xE0u + k);
		} else {
			hostTickSet(hostTickPeek() + 2);
			(void)SPI1->SR;
		}
	}
	CHECK((SPI1->SR & 0x41u) == 0u);  // RXNE (bit 0) and OVR (bit 6)
}

TEST_CASE_FIXTURE(SpiFixture, "RV7 SPI full duplex transmit receives the frames (control, P12C-T4A-001)") {
	SPI1->CR1 = 0x344u;  // master, SPE, SSM, SSI
	for (unsigned k = 0; k < 2; ++k) {
		SPI1->DR = 0x5Au + k;
		hostTickSet(hostTickPeek() + 2);
		(void)SPI1->SR;
	}
	CHECK((SPI1->SR & 0x41u) == 0x41u);  // the second frame overran the first one, which nobody read
}
