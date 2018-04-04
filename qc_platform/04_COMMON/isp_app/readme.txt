** Below steps are used for Boot loader v08 and later
Use ISP application to flash new firmware
[Linux]
1. Connect micro USB to FT2232 port on ASB.
2. Turn off minicom or miniterm.
3. Use FT2232_GPIO (latest version on Google Drive) to set boot loader to UART program mode
   $sudo ./FT2232_GPIO uart
4. Use isp_app_usart to flash following below formular:
   $sudo ./isp_app_usart -i usart -m program -f <file name> -b /dev/ttyUSBn
5. After flashing success, boot loader will run automatically.