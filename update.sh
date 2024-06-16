#!/bin/bash

root_folder=~/models1/
cd $root_folder
dir_pattern="s[0-9]*"
matched_dirs=($(ls -d $dir_pattern))

# check if there is matched dirs
if [ ${#matched_dirs[@]} -eq 0 ]; then
    echo "No matching directory found."
    exit 1
elif [ ${#matched_dirs[@]} -gt 1 ]; then
    echo "More than one matching directory found. Please handle manually."
    exit 1
fi

# get matched dirs
for original_dir in $matched_dirs;
do
    # extract number
    dir_name="${original_dir#s}"
    number_part="${dir_name%%[^0-9]*}"

    # get the new number
    new_number=$((number_part + 1))
    new_dir="s$new_number"
    older_number=$((number_part - 1))
    older_dir="s$older_number"
    echo "new_dir: $new_dir, original_dir: $original_dir, older_dir: $older_dir"

    echo "Renaming directory '$original_dir' to '$new_dir'"
    mv "$original_dir" "$new_dir"

    cd $root_folder/$new_dir
    sed -i 's/DR_MODEL_PREFIX=$original_dir/DR_MODEL_PREFIX=$new_dir/g' run.env
    sed -i 's/DR_LOCAL_S3_PRETRAINED_PREFIX=training\/\$DR_WORLD_NAME\/$older_dir/DR_LOCAL_S3_PRETRAINED_PREFIX=training\/\$DR_WORLD_NAME\/$original_dir/g' run.env

done
echo "complete."