echo start downloading
set uv4_path="D:\Keil_v5\UV4\UV4"
cd %1\projects\gigadevice\gd32450z_eval\uvision\aws_tests
%uv4_path% -j0 -f GD32450Z_EVAL.uvprojx
exit 0