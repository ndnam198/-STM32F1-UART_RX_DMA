#!/bin/sh
timestamp=$(date +"%D %T")
message=$1
git add -A;
git commit -m "[$timestamp]: $message"
git push https://github.com/ndnam198/STM32F1-UART_RX_DMA.git UART_CLI
