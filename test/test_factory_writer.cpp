#include <gtest/gtest.h>
#include "FactoryWriter.hpp"

namespace rosneuro {

TEST(FactoryWriterTest, CreateWriterTest) {
    FactoryWriter factory;
    NeuroFrame frame;

    std::unique_ptr<Writer> xdfwrt   = factory.createWriter(&frame, WriterType::XDFWRT);
    std::unique_ptr<Writer> dummywrt = factory.createWriter(&frame, WriterType::DUMMYWRT);
    std::unique_ptr<Writer> unknown = factory.createWriter(nullptr);

    EXPECT_NE(dynamic_cast<XDFWriter*>(xdfwrt.get()), nullptr);
    EXPECT_NE(dynamic_cast<DummyWriter*>(dummywrt.get()), nullptr);
    EXPECT_NE(dynamic_cast<Writer*>(unknown.get()), nullptr);
}

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}