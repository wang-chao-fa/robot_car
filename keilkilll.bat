@echo off
:: 清理编译中间文件 (保留源码)
del /s /q *.o 2>nul
del /s /q *.d 2>nul
del /s /q *.crf 2>nul
del /s /q *.axf 2>nul
del /s /q *.lnp 2>nul
del /s /q *.lst 2>nul
del /s /q *.map 2>nul
del /s /q *.dep 2>nul
del /s /q *.htm 2>nul
del /s /q JLinkLog.txt 2>nul
echo 清理完成!
pause
