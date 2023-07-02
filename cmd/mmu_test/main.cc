// sim5
// Copyright (c) 2022-2023, Joshua Scoggins
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// The main for the arduino version of the simulator's main
// Created by jwscoggins on 8/21/21.
//
#include <iostream>
#include <random>
#include <chrono>
using Address = uint32_t;
struct SegmentDescriptor {
    uint32_t unused[2];
    union {
        uint32_t full;
        struct {
            uint32_t offset : 6;
            uint32_t baseAddress : 26;
        } generic;

    } address;
    union {
        uint32_t full;
        struct {
            uint32_t valid : 1;
            uint32_t pagingMethod : 2;
            uint32_t accessStatus : 5;
            uint32_t unused0 : 10;
            uint32_t size : 6;
            uint32_t unused1 : 4;
            uint32_t type : 4;
        } generic;
    } info;
};
static_assert(sizeof(SegmentDescriptor) == (sizeof(uint32_t) * 4));
struct MMUAddress {
    explicit constexpr MMUAddress(Address value) : full(value) { }
    constexpr auto getFullAddress() const noexcept { return full; }
    constexpr auto getPageOffset() const noexcept { return offset; }
    constexpr auto getRegion() const noexcept { return region; }
    constexpr auto getPageTableOffset() const noexcept { return pageTableOffset; }
    constexpr auto getPageTableDirectoryOffset() const noexcept { return pageTableDirectoryOffset; }
    constexpr auto isSimpleRegion() const noexcept { return simpleRegionView.unused == 0; }
    constexpr auto isValidPagedRegion(Address size) const noexcept {
        return pagedRegionView.pageTableOffset < size;
    }
    constexpr auto isValidBipagedRegion(Address size) const noexcept {
        return pageTableDirectoryOffset < size;
    }
    constexpr auto isPagedRegion() const noexcept { return pagedRegionView.unused == 0; }
    union {
        Address full;
        struct {
            Address offset: 12;
            Address pageTableOffset: 10;
            Address pageTableDirectoryOffset: 8;
            Address region: 2;
        };
        struct {
            Address offset : 12;
            Address unused : 18;
            Address region : 2;
        } simpleRegionView;
        struct {
            Address offset : 12;
            Address pageTableOffset : 10;
            Address unused : 8;
            Address region : 2;
        } pagedRegionView;
        struct {
            Address offset : 12;
            Address baseAddress : 20;
        } virtualAddress;
    };
};
void
testOutMMUAddressTranslation() {
    std::cout << "Generating 255 random addresses and determining information based off the address alone" << std::endl;
    std::mt19937 generator;
    generator.seed(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    // test out address decomposition
   for (int i = 0; i < 255; ++i)  {
       MMUAddress addr(generator());
       std::cout << "Virtual Address: 0x" << std::hex << addr.getFullAddress() << std::endl;
       std::cout << "\tKind: ";
       if (addr.isSimpleRegion()) {
           std::cout << "\tSimple Region" << std::endl;
       } else if (addr.isPagedRegion()) {
           std::cout << "\tPaged Region" << std::endl;
       } else {
           std::cout << "\tBipaged Region" << std::endl;
       }
       std::cout << "\tMajor Region: 0x" << std::hex << addr.getRegion() << std::endl;
       std::cout << "\tPage Table Directory Offset: 0x" << std::hex << addr.getPageTableDirectoryOffset() << std::endl;
       std::cout << "\tPage Table Offset: 0x" << std::hex << addr.getPageTableOffset() << std::endl;
       std::cout << "\tOffset: 0x" << std::hex << addr.getPageOffset() << std::endl;
   }
}
int
main(int argc, char** argv) {
    testOutMMUAddressTranslation();
    return 0;
}

