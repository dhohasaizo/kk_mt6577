kk_mt6577
original from LGD685

1. Android build
  - Download original android source code ( kitkat 4.4.2 ) from http://source.android.com
  - And, merge the source into the android source code
  - Run following scripts to build android
    a) source build/envsetup.sh
    b) lunch 1
    c) make
  - When you compile the android source code, you have to add google original prebuilt source(toolchain) into the android directory.
  - After build, you can find output at out/target/product/generic

2. Kernel Build  
  - Run in your directory source kernel
  - export your toolchain
  - Run following scripts to build kernel
  		 a) ./mk muse77_phone_kk n k  

  - After build, you can find the build image at out/target/product/muse77_phone_kk/

3. how to build chromium_lge (vendor\lge\external\chromium_lge\src),
       please refer to README.txt at the folder mentioned above.



