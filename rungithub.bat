@REM blame Daniel Sabalakov
@REM basic batch commit script that pushes everything. Makes life easier.
@echo off
git add *
git commit -m "Changes made on %date% at %time%"
@echo on
git push
@echo off
git pull