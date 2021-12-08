#include "QrToPng.h"

int main() {
    
        std::string qrText = "id1";
        std::string fileName = "QR_1.png";

        int imgSize = 300;
        int minModulePixelSize = 3;
        auto QrPng = QrToPng(fileName, imgSize, minModulePixelSize, qrText, true, qrcodegen::QrCode::Ecc::MEDIUM);

        if (QrPng.writeToPNG())
            std::cout << "Success!" << std::endl;
        else
            std::cerr << "Failure..." << std::endl;       



    return 0;
}