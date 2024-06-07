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
#include "Core.h"
#include <iostream>
#include <boost/program_options.hpp>
#include <filesystem>
#include <fstream>
#include <elfio/elfio.hpp>


int
main(int argc, char** argv) {
    Core core;
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
    std::cout << "i960 Simulator System" << std::endl;
    std::cout << "(C) 2022-2024 Joshua Scoggins" << std::endl;
    try {
        core.begin();
        if (vm.count("bootloader")) {
            auto path = vm["bootloader"].as<std::filesystem::path>();
            if (!std::filesystem::exists(path)) {
                std::cout << "File: "  << path << " does not exist!" << std::endl;
                return 1;
            } else if (std::filesystem::is_directory(path)) {
                std::cout << "File: " << path << " is a directory!" << std::endl;
            }

            // okay so we know it is a valid file and not a directory
            ELFIO::elfio reader;
            if (reader.load(path)) {
                if (reader.get_machine() != ELFIO::EM_960) {
                    std::cout << "provided ELF is not an i960 ELF!" << std::endl;
                    return 1;
                } 
                // we need to read each section and look for the important
                // parts to install into memory
                // there are three sections we absolutely need
                // they are .boot_words, .text, and .bss
                for (const auto& section : reader.sections) {
                    std::string name{section->get_name()};
                    auto baseAddress = section->get_address();
                    auto size = section->get_size();
#if 0
                    std::cout << "section: " << name<< std::endl;
                    std::cout << "\tbase address: 0x" << std::hex << baseAddress << std::endl;
                    std::cout << "\tsize: 0x" << std::hex << size << std::endl;
#endif
                    if (section->get_flags() & ELFIO::SHF_ALLOC) {
                        switch (section->get_type()) {
                            case ELFIO::SHT_PROGBITS:
                                installToMainMemory(baseAddress, section->get_data(), size);
                                break;
                            case ELFIO::SHT_NOBITS:
                                clearMainMemory(baseAddress, size);
                                break;
                            default:
                                break;

                        }
                    }
                }
            } else {
                std::cout << "could not open " << path << std::endl;
                return 1;
            }

        } else {
            std::cout << "No bootloader provided! Running with memory completely empty!" << std::endl;
        }

        // setup the memory image as needed
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

