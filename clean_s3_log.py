import boto3

def delete_folder_from_s3(bucket_name, folder_prefix_list):
    """
    Deletes a folder and its contents from the specified S3 bucket.
    
    :param bucket_name: The name of the S3 bucket.
    :param folder_prefix: The prefix of the folder to delete.
    """
    # Initialize the S3 client
    s3 = boto3.client('s3')
    for folder_prefix in folder_prefix_list:
    
        # List objects in the bucket
        paginator = s3.get_paginator('list_objects_v2')
        keys_to_delete = []
        for result in paginator.paginate(Bucket=bucket_name, Prefix=folder_prefix):
            # Collect keys to delete
            for key in result.get('Contents', []):
                keys_to_delete.append({'Key': key['Key']})
        
        # Delete the collected keys
        if keys_to_delete:
            # Batch delete the keys
            s3.delete_objects(Bucket=bucket_name, Delete={'Objects': keys_to_delete})

        print('delete bucket {}'.format(folder_prefix))

# Example usage
bucket_name = 'mariox-base-bucket-gffnrjtkqlw6'
world_name = 'arctic_open_ccw'
model_name = 'aug12'
folder_prefix_list = ['training/{}/{}'.format(world_name, model_name), 
                      'upload/{}/{}'.format(world_name, model_name), 
                      'custom_files/{}/{}'.format(world_name, model_name)]

delete_folder_from_s3(bucket_name, folder_prefix_list)