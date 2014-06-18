tesseract-ocr 3.03 and leptonica 1.70 setup 

1. Installed tesseract_3.03.02.orig.tar.gz from https://launchpad.net/ubuntu/+source/tesseract/3.03.02-3

2. Extracted that thing
    tar -xvzf tesseract_3.03.02.orig.tar.gz

3. As said on the Compiling section
    sudo apt-get install autoconf automake libtool
    sudo apt-get install libpng12-dev
    sudo apt-get install libjpeg62-dev
    sudo apt-get install libtiff4-dev
    sudo apt-get install zlib1g-dev
    sudo apt-get install libicu-dev      # (if you plan to make the training tools)
    sudo apt-get install libpango1.0-dev # (if you plan to make the training tools)
    sudo apt-get install libcairo2-dev   # (if you plan to make the training tools)

4. Download leptonica 1.70 from http://www.leptonica.org/download.html

5. Extracted leptonica thing
    tar -xvzf leptonica-1.70.tar.gz 

6. As suggested on http://www.leptonica.org/source/README.html...
    ./configure 
    make 
    make install

7. Back to tesseract things!
    ./autogen.sh
    ./configure
    make
    sudo make install
    sudo ldconfig