import boto3
import os

def download_logs_from_s3(bucket_name, prefix, local_directory):
    # Initialize the S3 client
    s3 = boto3.client('s3')
    
    # Ensure the local directory exists
    if not os.path.exists(local_directory):
        os.makedirs(local_directory)
    
    # List objects in the S3 bucket
    paginator = s3.get_paginator('list_objects_v2')
    for result in paginator.paginate(Bucket=bucket_name, Prefix=prefix):
        # Download each log file
        for key in result.get('Contents', []):

            if 'robomaker' in key['Key']:
                # Construct the local path
                file_name = key['Key']
                local_path = os.path.join(local_directory, os.path.basename(file_name))
            
                # Download the file
                s3.download_file(bucket_name, file_name, local_path)
                print('downloaded file %s' % local_path)

# Example usage
model_name = 'aug1203'
local_log_folder = model_name[:5]
world_name = 'arctic_open_ccw'
bucket_name = 'mariox-base-bucket-gffnrjtkqlw6'
prefix = 'training/{}/{}/logs/'.format(world_name, model_name)  # Specify the prefix if your logs are in a specific folder
local_directory = 'C:\\Users\\PC\\workspace\\deepracer\\deepracer-log-guru\\logs\\' + local_log_folder

download_logs_from_s3(bucket_name, prefix, local_directory)

# running model:
# aug1501
# aug1201