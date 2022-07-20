@echo off
@cd /D "%~dp0"
@if exist "%*" (
	@echo file exists
	@set filepath=%*
) else (
	@set /p filepath="Data file: "
)

@if exist "%filepath%" (
	@copy /Y "%filepath%" .\data.bin >\nul
)

@start .\plotter_all.plt
@start .\plotter_p.plt
