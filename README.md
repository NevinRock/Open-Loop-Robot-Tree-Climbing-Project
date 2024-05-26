# Warning⚠️
**Please try this project only if you have some embedded development experience. Otherwise, there will be many unintelligible aspects and potential security risks!**
## Brief Introduction👲

This project focuses on developing an open-loop robot designed to climb trees. The robot uses a microcontroller to control its movement and various hardware components to achieve the climbing mechanism.

## Operating System📢

- Windows 11 Professional 23H2 22631.3593

## Microcontroller Models🌏

- [STM32F103C8T6](https://www.st.com/en/microcontrollers-microprocessors/stm32f103c8.html)

### Purchase Links for Development Board🛒

- [STM32F103C8T6 小系统板 STM32单片机开发板核心板入门套件 C6T6](https://detail.tmall.com/item.htm?abbucket=1&id=734040301724&ns=1&priceTId=2150449c17167574808597897e0bcc&spm=a21n57.1.item.4.314d523cZCoCg0)
    - _Burners can be purchased from the same web link_
- [STM32F103C8T6 ARM STM32 Minimum System Development Board Module For CH32F103C8T6](https://www.amazon.co.uk/STM32F103C8T6-Minimum-System-Development-CH32F103C8T6/dp/B0CQHPMKLK/ref=sr_1_10?crid=3DBZ91YFBZIZW&dib=eyJ2IjoiMSJ9.rBCOpvMxu44WArQ_1KcH_m1MgF4xLSAcbcd7_djmCET2SiyTMgV1qktHoHGYqQYIsYbVZ-YSbn0dAubGGx8LH08eaer-T4b4WxwMwT9WZYsgZU4oH58pRyIOR1J46pqvwP_8AudCeK55J4FikWetcl7qsRVOpKGjpvImHg3gtnh2fuyDfHvgrZnuNLhow_dF4Z3SFDVVkLmQgdqj3OKr-YCbIJR3VBQF6tgPlqdESCg.EHGqO1uGJTjg02HK7yJS3Mzm2TgE650SH-OSWqHSQzo&dib_tag=se&keywords=stm32f103c8t6&qid=1716757990&sprefix=stm32%2Caps%2C161&sr=8-10)
    - _Requires additional purchase of burner: [ST-Link](https://www.amazon.co.uk/AZDelivery-Programming-Aluminium-Downloader-Programmer/dp/B086TWZNMM/ref=pd_day0fbt_thbs_d_sccl_1/261-5086978-6273962?pd_rd_w=WjXod&content-id=amzn1.sym.bfe06781-69ca-4a3a-9842-e535d2f2ae0e&pf_rd_p=bfe06781-69ca-4a3a-9842-e535d2f2ae0e&pf_rd_r=XKF2ZQ8J9DFR6TRYAKVE&pd_rd_wg=0UZP4&pd_rd_r=e77ec29c-9f70-4c4f-b10d-b939ec91a0e5&pd_rd_i=B086TWZNMM&psc=1)_

## Hardware List👨‍👩‍👧

- [Parallax Standard Servo](https://www.parallax.com/product/parallax-standard-servo/)
- [EasyDriver - Stepper Motor Driver](https://www.sparkfun.com/products/12779)
- [RS PRO Hybrid, Permanent Magnet Stepper Motor, 60mNm Torque, 3.8 V, 1.8°, 28 x 28mm Frame, 5mm Shaft](https://uk.rs-online.com/web/p/stepper-motors/5350344)

## Begin🧑‍🍼

### Software Downloads

- Download [Keil](https://www.keil.com/download/product/)
- Download [Java](https://www.java.com/download/ie_manual.jsp)
- Download [CubeMX](https://www.st.com/content/st_com/en/stm32cubemx.html)

### How to Use

1. **Open** the project file: `term_3_auto_F1_Stepper_5_26\MDK-ARM\term_3_auto_F1_Stepper_5_26.uvprojx`
2. **Connect** your PC and the [Development Board](https://www.amazon.co.uk/STM32F103C8T6-Minimum-System-Development-CH32F103C8T6/dp/B0CQHPMKLK/ref=sr_1_10?crid=3DBZ91YFBZIZW&dib=eyJ2IjoiMSJ9.rBCOpvMxu44WArQ_1KcH_m1MgF4xLSAcbcd7_djmCET2SiyTMgV1qktHoHGYqQYIsYbVZ-YSbn0dAubGGx8LH08eaer-T4b4WxwMwT9WZYsgZU4oH58pRyIOR1J46pqvwP_8AudCeK55J4FikWetcl7qsRVOpKGjpvImHg3gtnh2fuyDfHvgrZnuNLhow_dF4Z3SFDVVkLmQgdqj3OKr-YCbIJR3VBQF6tgPlqdESCg.EHGqO1uGJTjg02HK7yJS3Mzm2TgE650SH-OSWqHSQzo&dib_tag=se&keywords=stm32f103c8t6&qid=1716757990&sprefix=stm32%2Caps%2C161&sr=8-10) with the [Burner](https://www.amazon.co.uk/STM32F103C8T6-Minimum-System-Development-CH32F103C8T6/dp/B0CQHPMKLK/ref=sr_1_10?crid=3DBZ91YFBZIZW&dib=eyJ2IjoiMSJ9.rBCOpvMxu44WArQ_1KcH_m1MgF4xLSAcbcd7_djmCET2SiyTMgV1qktHoHGYqQYIsYbVZ-YSbn0dAubGGx8LH08eaer-T4b4WxwMwT9WZYsgZU4oH58pRyIOR1J46pqvwP_8AudCeK55J4FikWetcl7qsRVOpKGjpvImHg3gtnh2fuyDfHvgrZnuNLhow_dF4Z3SFDVVkLmQgdqj3OKr-YCbIJR3VBQF6tgPlqdESCg.EHGqO1uGJTjg02HK7yJS3Mzm2TgE650SH-OSWqHSQzo&dib_tag=se&keywords=stm32f103c8t6&qid=1716757990&sprefix=stm32%2Caps%2C161&sr=8-10).
3. **Click** `Download` or press `F8` to program the board.

