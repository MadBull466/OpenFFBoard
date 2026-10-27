/*
 * Tests of cpp_freertos::MemoryPool (FFBoard/Src/cmem_pool.cpp, FFBoard/Inc/mem_pool.hpp; tier A vendor code: suspected
 * defects are reported, never fixed) on the fake FreeRTOS API F11 (support/HostRtosExt.h).
 *
 * cmem_pool.cpp is compiled through platform/src/host_cmem_pool.cpp: its malloc() calls go to pvPortMalloc like on the
 * target (cppmain.cpp:138), so heap_4's pvPortMalloc(0) == NULL and injected heap failures
 * (HostRtos::failCreate(RtosObject::Heap)) reach it. HostRtos::heapStats() shows the requested sizes.
 *
 * ~MemoryPool is deleted (mem_pool.hpp:275): every pool is created with new and kept reachable with keepAlive.
 * Item addresses are compared with each other (items are handed out in address order, FreeItems is a FIFO list).
 *
 * Host vs target: the alignment is raised to sizeof(unsigned char *) (8 on the host, 4 on the target; NEEDS REVIEW
 * 143): tests that depend on it use sizeof(void *) or an alignment of at least 8. The exception paths are host-only
 * (the F407 defines CPP_FREERTOS_NO_EXCEPTIONS and stops in configASSERT; NEEDS REVIEW 142).
 */
#include "doctest.h"
#include "RtosFixture.h"
#include "TestHelpers.h"

#include "mem_pool.hpp"

#include <cstdint>
#include <cstring>

using namespace cpp_freertos;
using HostRtos::RtosObject;

namespace {

/** Distance in bytes between two items */
ptrdiff_t distance(void *from, void *to) { return static_cast<unsigned char *>(to) - static_cast<unsigned char *>(from); }

/** Number of items Allocate hands out until the pool is exhausted (the items are not returned) */
int drain(MemoryPool &pool) {
	int count = 0;
	while (pool.Allocate() != nullptr) {
		count++;
		REQUIRE(count < 100000);
	}
	return count;
}

} // namespace

TEST_SUITE("MemoryPool") {

// ---------------- Construction with malloc ----------------

TEST_CASE_FIXTURE(RtosFixture, "a pool allocates item count times the item size from the RTOS heap") {
	const uint32_t calls = HostRtos::heapStats().mallocCalls;
	MemoryPool *pool = keepAlive(new MemoryPool(16, 3, 8));
	CHECK(HostRtos::heapStats().mallocCalls == calls + 1);
	CHECK(HostRtos::heapStats().lastMallocSize == 48);
	CHECK(drain(*pool) == 3);
}

TEST_CASE_FIXTURE(RtosFixture, "the items of a pool lie one item size apart in the allocated block") {
	MemoryPool *pool = keepAlive(new MemoryPool(16, 3, 8));
	void *block = HostRtos::heapStats().lastMallocResult;
	void *first = pool->Allocate();
	void *second = pool->Allocate();
	void *third = pool->Allocate();
	CHECK(first == block);
	CHECK(distance(first, second) == 16);
	CHECK(distance(second, third) == 16);
}

TEST_CASE_FIXTURE(RtosFixture, "an item size below the alignment is raised to the alignment") {
	MemoryPool *pool = keepAlive(new MemoryPool(1, 2, 16));
	CHECK(HostRtos::heapStats().lastMallocSize == 32);
	void *first = pool->Allocate();
	void *second = pool->Allocate();
	CHECK(distance(first, second) == 16);
}

TEST_CASE_FIXTURE(RtosFixture, "an item size above the alignment is rounded up to the next multiple of the alignment") {
	MemoryPool *pool = keepAlive(new MemoryPool(17, 2, 8));
	CHECK(HostRtos::heapStats().lastMallocSize == 48);
	void *first = pool->Allocate();
	void *second = pool->Allocate();
	CHECK(distance(first, second) == 24);
}

TEST_CASE_FIXTURE(RtosFixture, "an item size that is a multiple of the alignment is kept") {
	MemoryPool *pool = keepAlive(new MemoryPool(32, 2, 8));
	CHECK(HostRtos::heapStats().lastMallocSize == 64);
	void *first = pool->Allocate();
	void *second = pool->Allocate();
	CHECK(distance(first, second) == 32);
}

TEST_CASE_FIXTURE(RtosFixture, "an alignment below the pointer size is raised to the pointer size") {
	// Host vs target: sizeof(unsigned char *) is 8 on the host and 4 on the target (NEEDS REVIEW 143)
	MemoryPool *pool = keepAlive(new MemoryPool(1, 2, 2));
	CHECK(HostRtos::heapStats().lastMallocSize == 2 * sizeof(void *));
	void *first = pool->Allocate();
	void *second = pool->Allocate();
	CHECK(distance(first, second) == static_cast<ptrdiff_t>(sizeof(void *)));
}

TEST_CASE_FIXTURE(RtosFixture, "an alignment that is not a power of two throws MemoryPoolBadAlignmentException before allocating") {
	const uint32_t calls = HostRtos::heapStats().mallocCalls;
	CHECK_THROWS_WITH_AS(keepAlive(new MemoryPool(8, 2, 24)), "MemoryPool Bad Alignment", MemoryPoolBadAlignmentException);
	CHECK(HostRtos::heapStats().mallocCalls == calls);
}

TEST_CASE_FIXTURE(RtosFixture, "the largest power of two alignment of an int is accepted") {
	static unsigned char buffer[8];
	MemoryPool *pool = nullptr;
	CHECK_NOTHROW(pool = keepAlive(new MemoryPool(1, buffer, 0, 1 << 30)));
	REQUIRE(pool != nullptr);
	CHECK(pool->Allocate() == nullptr); // Item size raised to 2^30: nothing fits
}

TEST_CASE_FIXTURE(RtosFixture, "a pool whose memory can not be allocated throws MemoryPoolMallocException and creates no mutex") {
	const size_t semaphores = HostRtos::semaphoreCount();
	const size_t blocks = HostRtos::heapLiveBlocks();
	HostRtos::failCreate(RtosObject::Heap);
	CHECK_THROWS_WITH_AS(keepAlive(new MemoryPool(8, 4, 8)), "MemoryPool malloc Failed", MemoryPoolMallocException);
	CHECK(HostRtos::semaphoreCount() == semaphores);
	CHECK(HostRtos::heapLiveBlocks() == blocks);
}

TEST_CASE_FIXTURE(RtosFixture, "characterization: a pool with an item count of 0 can not be created because the RTOS heap returns NULL for 0 bytes") {
	// NEEDS REVIEW 150: heap_4's pvPortMalloc(0) returns NULL (heap_4.c:141), so MemoryPool(size, 0, alignment) throws
	// (configASSERT on the target) although an empty pool filled later with AddMemory looks like a valid use. Intended?
	CHECK_THROWS_AS(keepAlive(new MemoryPool(8, 0, 8)), MemoryPoolMallocException);
	CHECK(HostRtos::heapStats().lastMallocSize == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "a pool creates its lock mutex") {
	const size_t semaphores = HostRtos::semaphoreCount();
	keepAlive(new MemoryPool(8, 1, 8));
	CHECK(HostRtos::semaphoreCount() == semaphores + 1);
	CHECK(HostRtos::createCalls(RtosObject::Mutex) == 1);
}

// ---------------- Allocate and Free ----------------

TEST_CASE_FIXTURE(RtosFixture, "an exhausted pool returns NULL until an item is freed") {
	MemoryPool *pool = keepAlive(new MemoryPool(8, 1, 8));
	void *item = pool->Allocate();
	REQUIRE(item != nullptr);
	CHECK(pool->Allocate() == nullptr);
	pool->Free(item);
	CHECK(pool->Allocate() == item);
}

TEST_CASE_FIXTURE(RtosFixture, "Allocate and Free leave the pool lock released") {
	MemoryPool *pool = keepAlive(new MemoryPool(8, 2, 8));
	void *item = pool->Allocate();
	pool->Free(item);
	CHECK(pool->Allocate() != nullptr); // A held lock would make the next call wait forever (kernel fault)
	CHECK(pool->Allocate() != nullptr);
	CHECK(pool->Allocate() == nullptr); // The exhausted path also unlocks
	CHECK(pool->Allocate() == nullptr);
}

// ---------------- Construction with preallocated memory ----------------

TEST_CASE_FIXTURE(RtosFixture, "a pool on preallocated memory uses the buffer without allocating") {
	alignas(16) static unsigned char buffer[32];
	const uint32_t calls = HostRtos::heapStats().mallocCalls;
	MemoryPool *pool = keepAlive(new MemoryPool(16, buffer, sizeof(buffer), 8));
	CHECK(HostRtos::heapStats().mallocCalls == calls);
	CHECK(pool->Allocate() == buffer);
	CHECK(pool->Allocate() == buffer + 16);
	CHECK(pool->Allocate() == nullptr);
}

TEST_CASE_FIXTURE(RtosFixture, "a pool on preallocated memory rounds the item size to the alignment and ignores the rest of the buffer") {
	alignas(16) static unsigned char buffer[40];
	MemoryPool *pool = keepAlive(new MemoryPool(9, buffer, sizeof(buffer), 8));
	CHECK(pool->Allocate() == buffer);
	CHECK(pool->Allocate() == buffer + 16);
	CHECK(pool->Allocate() == nullptr); // 8 bytes left over
}

TEST_CASE_FIXTURE(RtosFixture, "a preallocated buffer smaller than one item gives an empty pool") {
	alignas(16) static unsigned char buffer[15];
	MemoryPool *pool = keepAlive(new MemoryPool(16, buffer, sizeof(buffer), 8));
	CHECK(pool->Allocate() == nullptr);
}

TEST_CASE_FIXTURE(RtosFixture, "the preallocated constructor also rejects an alignment that is not a power of two") {
	alignas(16) static unsigned char buffer[32];
	CHECK_THROWS_AS(keepAlive(new MemoryPool(8, buffer, sizeof(buffer), 3 * 8)), MemoryPoolBadAlignmentException);
}

// ---------------- AddMemory ----------------

TEST_CASE_FIXTURE(RtosFixture, "AddMemory with an item count allocates that many items of the pool's item size") {
	MemoryPool *pool = keepAlive(new MemoryPool(10, 1, 8));
	REQUIRE(pool->Allocate() != nullptr);
	pool->AddMemory(3);
	CHECK(HostRtos::heapStats().lastMallocSize == 48);
	void *block = HostRtos::heapStats().lastMallocResult;
	void *first = pool->Allocate();
	CHECK(first == block);
	CHECK(distance(first, pool->Allocate()) == 16);
	CHECK(drain(*pool) == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "AddMemory throws MemoryPoolMallocException when the heap is exhausted and keeps the existing items") {
	MemoryPool *pool = keepAlive(new MemoryPool(8, 2, 8));
	HostRtos::failCreate(RtosObject::Heap);
	CHECK_THROWS_AS(pool->AddMemory(4), MemoryPoolMallocException);
	CHECK(drain(*pool) == 2);
}

TEST_CASE_FIXTURE(RtosFixture, "characterization: AddMemory with an item count of 0 throws because the RTOS heap returns NULL for 0 bytes") {
	// NEEDS REVIEW 150 (see the constructor with an item count of 0)
	MemoryPool *pool = keepAlive(new MemoryPool(8, 1, 8));
	CHECK_THROWS_AS(pool->AddMemory(0), MemoryPoolMallocException);
	CHECK(drain(*pool) == 1);
}

TEST_CASE_FIXTURE(RtosFixture, "AddMemory with preallocated memory adds the items that fit") {
	alignas(16) static unsigned char buffer[40];
	MemoryPool *pool = keepAlive(new MemoryPool(16, 1, 8));
	REQUIRE(pool->Allocate() != nullptr);
	const uint32_t calls = HostRtos::heapStats().mallocCalls;
	pool->AddMemory(buffer, sizeof(buffer));
	CHECK(HostRtos::heapStats().mallocCalls == calls);
	CHECK(pool->Allocate() == buffer);
	CHECK(pool->Allocate() == buffer + 16);
	CHECK(pool->Allocate() == nullptr);
}

TEST_CASE_FIXTURE(RtosFixture, "AddMemory with a buffer of exactly one item adds that item") {
	alignas(16) static unsigned char buffer[16];
	MemoryPool *pool = keepAlive(new MemoryPool(16, buffer, 0, 8));
	pool->AddMemory(buffer, sizeof(buffer));
	CHECK(pool->Allocate() == buffer);
	CHECK(pool->Allocate() == nullptr);
}

TEST_CASE_FIXTURE(RtosFixture, "AddMemory leaves the pool lock released") {
	alignas(16) static unsigned char buffer[16];
	MemoryPool *pool = keepAlive(new MemoryPool(16, 1, 8));
	pool->AddMemory(1);
	pool->AddMemory(buffer, sizeof(buffer));
	CHECK(drain(*pool) == 3); // Allocate takes the lock: a lock left held would be a kernel fault here
}

// ---------------- Suspected vendor defects (tier A: reported, never fixed) ----------------

TEST_CASE_FIXTURE(RtosFixture, "the items of a pool are aligned to the requested alignment"
				  * KNOWN_ISSUE("vendor V16: MemoryPool never aligns the start of its memory (cmem_pool.cpp:136-143, also "
								":106-119 and AddMemory :172-204): items are only aligned to `alignment` if the memory passed "
								"in (or returned by malloc: heap_4 aligns to portBYTE_ALIGNMENT 8 on the F407) already is, "
								"although mem_pool.hpp:155-157/177-179 promise the alignment. The first failing assertion is "
								"the alignment check")) {
	alignas(64) static unsigned char buffer[128];
	MemoryPool *pool = keepAlive(new MemoryPool(16, buffer + 8, 96, 16));
	void *item = pool->Allocate();
	REQUIRE(item != nullptr);
	REQUIRE_ALL_RTOS_MODELLED();
	CHECK(reinterpret_cast<uintptr_t>(item) % 16 == 0);
}

TEST_CASE_FIXTURE(RtosFixture, "a pool whose lock mutex can not be created frees its memory"
				  * KNOWN_ISSUE("vendor V17: the MemoryPool constructor (cmem_pool.cpp:106, 121) allocates the item memory "
								"before `new MutexStandard()`; when the mutex creation throws MutexCreateException the block is "
								"never freed (memory leak, exception builds only: the F407 stops in configASSERT). The first "
								"failing assertion is the heap block check")) {
	const size_t blocks = HostRtos::heapLiveBlocks();
	HostRtos::failCreate(RtosObject::Mutex);
	CHECK_THROWS_AS(keepAlive(new MemoryPool(8, 4, 8)), MutexCreateException);
	(void)expectKernelFault("vSemaphoreDelete(NULL)"); // vendor V3 (~Mutex after the failed create), see test_csync_gaps.cpp
	REQUIRE_ALL_RTOS_MODELLED();
	CHECK(HostRtos::heapLiveBlocks() == blocks);
}

TEST_CASE_FIXTURE(RtosFixture, "a pool whose total size does not fit in an int is rejected with MemoryPoolMallocException"
				  * KNOWN_ISSUE_UB("vendor V18: `ItemSize * itemCount` (cmem_pool.cpp:106, also AddMemory :172) is an int "
								   "multiplication: 65536 * 65537 overflows (UB; wraps to 65536 here), malloc gets a 64 KiB "
								   "block and the constructor queues 65537 items, all but the first beyond the block (heap "
								   "overflow when used). Same on the 32-bit target. The first failing assertion is "
								   "CHECK_THROWS_AS")) {
	CHECK_THROWS_AS(keepAlive(new MemoryPool(65536, 65537, 8)), MemoryPoolMallocException);
}

}
