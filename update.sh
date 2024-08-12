#!/bin/bash

root_folder=~/models/
cd $root_folder
matched_dirs=($(ls -d aug[0-9]*))

# check if there is matched dirs
if [ ${#matched_dirs[@]} -eq 0 ]; then
    echo "No matching directory found."
    exit 1
fi

# get matched dirs
for original_dir in "${matched_dirs[@]}";
do
    # extract number
    dir_name="${original_dir#s}"
    number_part="${dir_name%%[^0-9]*}"

    # get the new number
    new_number=$((number_part + 1))
    new_dir="aug$new_number"
    older_number=$((number_part - 1))
    older_dir="aug$older_number"
    echo "+ new_dir: $new_dir, original_dir: $original_dir, older_dir: $older_dir"

    echo "+ Renaming directory '$original_dir' to '$new_dir'"
    mv "$original_dir" "$new_dir"

    echo "# Updating run.env in folder $new_dir"
    sed -i "s/$original_dir/$new_dir/g" $root_folder/$new_dir/run.env
    echo "- updated $original_dir to $new_dir"
    sed -i "s/$older_dir/$original_dir/g" $root_folder/$new_dir/run.env
    echo "- updated $older_dir to $original_dir"
done
echo "complete."
