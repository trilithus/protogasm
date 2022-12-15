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

@setlocal EnableExtensions
@set "uniquePath=temp\%RANDOM%"
@mkdir %uniquePath%

@copy /b data.bin %uniquePath%
@copy /b *.plt %uniquePath%
@pushd %uniquePath%
@start .\plotter_all.plt
@start .\plotter_p.plt
@popd
