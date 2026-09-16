#!/bin/bash
echo "--- files in teamcode dir ---"
ls org/firstinspires/ftc/teamcode/
echo "--- first 15 lines of compile output ---"
javac -d out org/firstinspires/ftc/teamcode/*.java 2>&1 | head -15
