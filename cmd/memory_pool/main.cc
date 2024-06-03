//
// Created by jwscoggins on 6/3/24.
//

#include <iostream>
#include <boost/asio.hpp>
#include <boost/program_options.hpp>


int main(int argc, char** argv) {
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
                ("help,h", "produce help message")
            ;
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }
    return 0;
}