#include "rosneuro_recorder/FactoryWriter.hpp"
#include <unistd.h>

using namespace rosneuro;

int main(int argc, char** argv) {
	rosneuro::NeuroFrame frame;
	FactoryWriter factory;

	std::unique_ptr<Writer> xdfwrt   = factory.createWriter(&frame, WriterType::XDFWRT);
	std::unique_ptr<Writer> dummywrt = factory.createWriter(&frame, WriterType::DUMMYWRT);
	
	xdfwrt->Who();
	dummywrt->Who();

	if(!xdfwrt->Open("./test_file.gdf")) {
        std::cerr << "Open failed" << std::endl;
    }

	if(!xdfwrt->Setup()) {
        std::cerr << "Setup failed" << std::endl;
    }

	return 0;
}
