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
#include "Types.h"
#include "BinaryOperations.h"
#include "Core.h"
#include <iostream>
#include <elfio/elfio.hpp>
#include <boost/program_options.hpp>
#include <filesystem>

int
main(int argc, char** argv) {
    Core core;
    std::cout << "i960 Simulator System" << std::endl;
    std::cout << "(C) 2022-2023 Joshua Scoggins" << std::endl;
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("bootloader", boost::program_options::value<std::filesystem::path>(), "bootstrap elf program")
            ;
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }
    if (vm.count("bootloader")) {
        ELFIO::elfio reader;
        auto path = vm["bootloader"].as<std::filesystem::path>();
        if (!reader.load(path)) {
            std::cout << "Could not process " << path << std::endl;
            return 2;
        }
        std::cout << "ELF file class: ";
        if (reader.get_class() == ELFIO::ELFCLASS32) {
            std::cout << "ELF32" << std::endl;
        } else {
            std::cout << "ELF64" << std::endl;
        }
        std::cout << "File encoding: ";
        if (reader.get_encoding() == ELFIO::ELFDATA2LSB) {
            std::cout << "Little endian" << std::endl;
        } else {
            std::cout << "Big endian" << std::endl;
        }
        std::cout << "Target Architecture: ";
        if (reader.get_machine() == ELFIO::EM_960) {
            std::cout << "i960" << std::endl;
        } else {
            std::cout << "unknown (" << reader.get_machine() << ")" << std::endl;
        }
        /// @todo install the bootloader image into main memory
    } else {
        std::cout << "No bootloader provided! Running with memory completely empty!" << std::endl;
    }

    // setup the memory image as needed
    try {
        core.begin();
        switch (core.start()) {
            case BootResult::SelfTestFailure:
                core.selfTestFailure();
                break;
            case BootResult::ChecksumFail:
                core.checksumFail();
                break;
            default:
                break;
        }
        while (true) {
            core.cycle();
        }
    } catch(std::runtime_error& ex) {
        std::cout << ex.what() << std::endl;
    }
    return 0;
}

