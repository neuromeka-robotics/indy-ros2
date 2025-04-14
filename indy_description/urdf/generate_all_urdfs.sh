#!/bin/bash

# List of indy types
indy_types=("indy7" "indy7_v2" "indy12" "indy12_v2" "indyrp2" "indyrp2_v2" "icon7l" "icon3" "nuri3s" "nuri4s" "nuri20c" "opti5" "nuri7c")

script_dir=$(cd "$(dirname "$0")" && pwd)
output_dir="${script_dir}/../urdf_files"

for indy_type in "${indy_types[@]}"
do
    if [[ "$indy_type" == "indy7" || "$indy_type" == "indyrp2" ]]; then
        for indy_eye in "true" "false"
        do
            if [ "$indy_eye" = "true" ]; then
                output_file="${output_dir}/${indy_type}_eye.urdf"
            else
                output_file="${output_dir}/${indy_type}.urdf"
            fi
            
            echo "Generating URDF for ${indy_type} with indy_eye=${indy_eye}..."
            ros2 run xacro xacro indy.urdf.xacro -o ${output_file} indy_type:=${indy_type} indy_eye:=${indy_eye} name:=indy
            
            if [ $? -eq 0 ]; then
                echo "URDF file generated successfully: ${output_file}"
            else
                echo "Failed to generate URDF file for ${indy_type} with indy_eye=${indy_eye}."
            fi
        done
    else
        output_file="${output_dir}/${indy_type}.urdf"
        echo "Generating URDF for ${indy_type}..."
        ros2 run xacro xacro indy.urdf.xacro -o ${output_file} indy_type:=${indy_type} indy_eye:=false name:=indy
        
        if [ $? -eq 0 ]; then
            echo "URDF file generated successfully: ${output_file}"
        else
            echo "Failed to generate URDF file for ${indy_type}."
        fi
    fi
done

echo "All URDF files have been generated."
