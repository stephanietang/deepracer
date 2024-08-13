# deepracer

## 2024 June

| Track Name | Numpy Files |
| ----- | ----- |
| Cosmic Loop | jyllandsringen_open |

| Model | Training Time(min) | reward function |hyperparameter | lap time | off track | remarks |
| ----- | ----- | ----- | ----- | ----- | ----- | ----- |
| s01 | 120 | TIF | lr=0.0002 |  01min13s | 22 | discarded |
| s02 | 240 | TIF | lr=0.0002 |  44s | 8 | discarded |
| s04 | 480 | TIF | lr=0.0005 |  01min12s | 22 | discarded, ir too large |
| s05 | 240 | TIF | lr=0.0005 |  42s | 9 | discarded, ir too large |
| s06 | 240 | reward_function_speed.py | lr=0.0002 |  29.666s | 3 |  |
| s07 | 240 | reward_function_speed.py | lr=0.0002 |  25.733s | 2 |  |
| s08 | 240 | reward_function_speed.py | lr=0.0002 |  31.476s | 2 | modify action space, adding more action spaces, DEEP -> SHALLOW, reward_function_speed.py modify step_reward thredshold |
| s09 <br> s09-1 | 240 <br> 240 | reward_function_minimalist.py | lr=0.0002 |  28.144s | 2 |  43% 75% |
| s10 | 240 | reward_function_minimalist_speed.py | lr=0.0002 | 2 mins plus | 41 | discarded, there is a bug in this reward function  |
| s11 | 180 | reward_function_speed.py | default | 38.899s | 7 | space action used in TIF, change DEEP to SHALLOW(default) 22.5%|
| **s12 <br> s12-1 <br> s12-2<br> s12-3<br> s1204** | 180<br>180<br>180<br>180<br>120 | reward_function_angle.py | default |  27.545s | 1 | action space used in TIF, stable model, average 75%, max 100%, min time during training is 22.581s, it is able to finish the laps without off track | 
| s13 | 180 | reward_function_angle2.py | default |  27.873s | 1 | 20% |
| s14 | 180 | reward_function_minimalist.py | default |  36.953s | 5 | 12% |
| s15 | 180 | reward_function_steps_progress.py | default |  46.743s | 10 |20% |
| s16 | 180 | reward_function_minimalist_speed.py | default |  59.930s | 17 | discarded, there is a bug in this reward function |
| **s17** | 170 | reward_function_encouraging_racing_line.py | default |  30.327s | 1 | easy to train, stable 35% |
| s18 | 480 | reward_function_optimal_trace2.py | default |   |  | use Capstone_AWS_DeepRacer to calculate the optimal paths and optimal space actions model_metadata_optimal.json use 2.5 as min speed, 4 as max speed. There is bug on it then discard this version  |
| s19 | 480 | reward_function_optimal_trace2.py | lr=0.0005 |   |  | There is bug on it then discard this version  |
| s20 | 480 | reward_function_optimal_trace2.py | lr=0.001<br>epocs=5<br>batch=32 |   |  | There is bug on it then discard this version |
| s21 | 480 | reward_function_optimal_trace2.py | default |   |  | There is bug on it then discard this version  |
| s22<br>s22-1 | 120<br>120 | reward_function_optimal_trace5.py | lr=0.0005 |   |  | remove steps_reward, add debugging log for first_racingpoint_index for each step <br>trained 4 hours average progress 12.7% max progress 43% |
| s23 | 120 | reward_function_optimal_trace5.py | default |   |  | remove steps_reward, add debugging log for first_racingpoint_index for each step <br>trained 4 hours average progress 11% max progress 46% |
| s24 | 120 | reward_function_optimal_trace5.py | lr=0.001<br>epochs=5<br>batch=32 |   |  | same as s23 trained 2 hours, average progress 6.5% max progress 30% too bad discarded |5
| s25<br>s25-1 | 120<br>120 | reward_function_optimal_trace5.py | discount factor=0.5 |   |  | average progress 9% max progress 40% too bad discarded |
| s26 | 120 | reward_function_optimal_trace5.py | default |   |  | trained on top of s23, but the progress is still very small, average progress: 14% max progress: 49% |
| **s27<br>s27-1** | 240<br>240 | reward_function_steps_progress | TIF hp |   |  | easy to train 37% 85% |
| **s28<br>s28-1** | 240<br>240 | reward_function_speed.py | TIF hp |   |  | retrain s07 to see if the model performance is stable 36% 90% |
| s29 | 180 | reward_function_angle.py | default |   |  | use model_metadata_optimal_max_5_min_2.json |
| s30<br>s30-1<br>s30-2 | 120<br>120<br>480 | reward_function_progress_velocity.py | default |   |  | use model_metadata_7_AS.json failed at s30-2 too long time, discarded, 16%, 50%|
| s31<br>s31-1<br>s31-2<br>s31-3 | 120<br>120<br>120<br>120 | my version of reward_function_progress_velocity.py | default |   |  | use model_metadata_optimal_max_4_min_2.json |
| s32<br>s32-1<br>s3202 | 30<br>120<br>120 | reward_function_optimal_trace5.py | default |   |  | use model_metadata_optimal_max_4_min_2.json |


## 2024 Aug

| Track Name | Numpy Files |
| ----- | ----- |
| Hot Rod Speedway | arctic_open |

- aug01: center line reward function, trained on arctic_open_ccw, trained 2 hours
- aug02: center line reward function, trained on arctic_open, since there is no arctic_open_ccw on logguru, trained 2 hours
- aug03: aug2024_arctic_open_follow_waypoints.py, account for reward for direction and distance reward, traide on arctic_open, trained for 2 hours
- aug04: aug2024_arctic_open_follow_waypoints.py, account only for distance reward, traide on arctic_open, trained for 2 hours
- aug05: aug2024_arctic_open_follow_waypoints.py, account only for distance reward, traide on arctic_open, trained for 5 mins, just want to verify the track waypoints by printing out the information - confirmed it is using arctic_open_ccw
- aug06: create a arctic_open_ccw track on log guru, default center line, trained on arctic_open_ccw, trained for 2 hours, use model_metadata_26_linear_AS_1.5_3.json as action space
- aug07: use model_metadata_26_linear_AS_1.5_3.json as action space and aug2024_arctic_open_ccw_follow_waypoints.py, it added speed_reward based on detecting turns(based on this repo https://github.com/MatthewSuntup/DeepRacer), trained for 3 hours, there is a bug in it, it doesn't take speed_reward into consideration, thus need to re-train, training also failed
- aug08: use model_metadata_26_linear_AS_1.5_3.json as action space and follow waypoints, fixed aug07 to consider speed_reward, trained for 3 hours, continue incremental training aug0802(2h), aug0803(2h), aug0804(2h), aug0805(2h), aug0806(1h), aug0807(90min) - why aug0807 is worse than aug0806?
- aug09, use model_metadata_18_linear_AS_2_4.json as action space and reward_function, follow waypoints, consider the distance_reward, direction_reward and progress reward, trained for 90mins, incremental training aug0901(1h), aug0902(1h, ir=0.001), aug0903(4h, ir=0.001, on standard instance)
- aug10, use model_metadata_18_linear_AS_2_4.json as action space and reward function look forward to future waypoints, change FUTURE_STEPS = 8 in inspired by completed lapse which cuts corner in aug0804, consider direction reward and progress reward, traied for 90mins, incremental training aug1001(4h, on standard instance)
- aug11, aug09 and aug10 don't converge on the first 3-4 hours, the reward function is not good and action space is too high, then stop them and start new training, use a new reward function which only focus on the distance to the optimal line and lower action space model_metadata_19_linear_AS_1.2_4_copy.json, start training for 5h
- aug12, use reward function as aug08 but use a different action space with faster speed model_metadata_19_linear_AS_1.5_4.json, train for 2h
- aug13, use aug2024_arctic_open_ccw_13 but use a different action space with faster speed model_metadata_19_linear_AS_1.5_4.json, train for 3h, to compare with aug12, incremental training aug1301(1h), aug1302(2h)
- aug14, use aug2024_arctic_open_ccw_13 and model_metadata_19_linear_AS_1.5_4.json, 3h, this reward function includes fine tuning the steering angle accountble for the curvature of the racing line


## Analysis on Reward Function

| Reward Function | Speed | Comment |
| ----- | ----- | ----- |
|reward_function_encouraging_racing_line|30s|easiest to train, 180mins training time to finish laps|
|reward_function_speed|25s|easy to train, 480mins to converge|
|reward_function_steps_progress||easy to train|
reward_function_angle|22s|relatively easy to train, fatest model so far|
|reward_function_progress_velocity||hard to train|
|reward_function_optimal_trace5||hardest to train|

## How to Run the scripts
- Change the models config under ./models
- change every time in run.env
    - `DR_LOCAL_S3_MODEL_PREFIX`
- Run the following command: `./auto_run.sh <time in mins>`
- It can trains multiple models at the same time if there are multiple models in ./models folder

## How to re-train the same model based on last checkpoint
- Run update.sh to update the models configuration under ./models, it will change `DR_LOCAL_S3_MODEL_PREFIX` and will point the pretrained model to last model name
- Run the following command: `./auto_run.sh <time in mins>`

## Useful links:

- Beginner:
    - Logging analysis
        - https://github.com/breadcentric/aws-deepracer-workshops/blob/enhance-log-analysis/Workshops/2018-reInvent/Lab1/Readme.md
        - https://aws.amazon.com/cn/blogs/machine-learning/using-log-analysis-to-drive-experiments-and-win-the-aws-deepracer-f1-proam-race/
    - hyper paramerters: 
        * https://medium.com/analytics-vidhya/aws-deepracer-looking-under-the-hood-for-design-of-the-reward-function-and-adjusting-e9dd3805ebbf
        * https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-console-train-evaluate-models.html#deepracer-define-action-space-for-training
        * https://catalog.workshops.aws/deepracer-200l/en-US/04-improving-your-aws-deepracer-model/01-hyperparameters
    - https://refactored.ai/microcourse/notebook?path=content%2FDeepRacer%2FAWS_DeepRacer_Reward_function_Additional_material.ipynb
    - https://www.linkedin.com/pulse/aws-deepracer-my-journey-from-17-seconds-95-dante-chen/
- Run faster by using custom waypoints: https://www.youtube.com/watch?v=__NjsBY2TS0
    - steps: https://www.linkedin.com/pulse/aws-deepracer-free-student-workshop-run-faster-using-your-cheuk-lam/?published=t
    - repo: https://github.com/oscarYCL/deepracer-waypoints-workshop/blob/main/2019example.py
- Calculate optimal racelines: https://www.youtube.com/watch?v=AdHhyvh0Bco
- Sample reward function: 
    - https://www.linkedin.com/pulse/samples-reward-functions-aws-deepracer-bahman-javadi/
    - https://github.com/scottpletcher/deepracer/tree/master/iterations
    - https://medium.com/@ajrberezowski/results-and-lessons-deepracer-student-league-march-2023-52c79121c3bb
    - https://medium.com/twodigits/aws-deepracer-how-to-train-a-model-in-15-minutes-3a0dca1175fb
    - https://github.com/Usin2705/DeepRacer
    - https://github.com/MatthewSuntup/DeepRacer
    - https://github.com/VilemR/AWS_DeepRacer/blob/master/reward_function.py

- understand the parameters in reward function
    - https://github.com/dmh23/deep_racer_framework
- An advanced Guide to AWS DeepRacer https://towardsdatascience.com/an-advanced-guide-to-aws-deepracer-2b462c37eea
- Breaking in to the Top 10 of AWS Deepracer Competition - May 2020 https://mickqg.github.io/DeepracerBlog/part2.html
- AWS DeepRacer event for EPAM: my approach to taking the 4th place https://medium.com/@rostyslav.myronenko/aws-deepracer-event-for-epam-my-approach-for-taking-the-4th-place-ff3f76f39b1e
- How we broke into the top 1% of the AWS DeepRacer Virtual Circuit https://blog.gofynd.com/how-we-broke-into-the-top-1-of-the-aws-deepracer-virtual-circuit-573ba46c275
- Craft a Powerful Reward Function for AWS DeepRacer Student League https://medium.com/@anshml/
- Learn AWS DeepRacer (tricks to optimize your reward function): https://medium.com/@syedbelalhyder/learn-aws-deepracer-tricks-to-optimize-your-reward-function-170af9d40602
- AWS Deepracer 2023: Racing towards New Insights: https://medium.com/@abhishekgupta97023/part-1-aws-deepracer-2023-racing-towards-new-insights-d383f815633e

- github:
    - deepracer-on-the-spot https://github.com/aws-deepracer-community/deepracer-on-the-spot
    - tracks: https://github.com/aws-deepracer-community/deepracer-race-data/blob/main/raw_data/tracks/README.md
    - deepracer-analysis: https://github.com/aws-deepracer-community/deepracer-analysis
    - deepracer-log-guru https://github.com/aws-deepracer-community/deepracer-log-guru
        - go to log guru directory, run `python -m src.main.guru` 
    - calculate the optimal racelines: https://github.com/cdthompson/deepracer-k1999-race-lines
    - calculate the optimal speed and action space: https://github.com/dgnzlz/Capstone_AWS_DeepRacer
    - waypoints visualization: https://github.com/ARCC-RACE/waypoint-visualization



## PCL Command running on local machine:
Please note this is for JPMC staff only
PURPOSE: This readme is to allow you to have AWS Service access via the command line for your sandbox.
         As part of this it provides you with temporary access credentials.
         This access is gained via the Sandbox Toolkit (pcl) and below are the instructions to get this.

PRE-REQUISITES:
- You have an RSA hard or soft token:
- You are working off-network on your own personal machine
- You have access to a command line tool - e.g. Git Bash


STEP 1: Download the latest pcl Toolkit (via the AWS Console)
- Login to the AWS Console via: https://idag2.jpmorganchase.com/adfs/ls/idpinitiatedsignon.aspx?logintorp=ASB
- (Ensure that you are working off-prem)
- (Ensure that you use your desktop password to login)
- In the AWS Console browser, open a new tab
- In this tab, open link: https://s3.console.aws.amazon.com/s3/buckets/l1-toolkit/?region=us-east-1&tab=overview
- Download the appropriate toolkit zip file
--- For Windows, pcl-..-windows-..-...zip
--- For linux, pcl-..-linux-..-...zip
--- For Mac, pcl-..-darwin-..-...zip
- Unzip this file

STEP 2: Run the pcl Toolkit
- Launch your command line tool (e.g. git bash)
- cd to the folder where you downloaded the pcl Toolkit
- ./pcl aws --sandbox-user --domain <your-domain> --sid <your-sid>  (optional) --profile-name <new_profile>
- INFO: Default profile name is 'adfs' - or you can be specific by appending '--profile-name <new-profile-name>''
- Enter your desktop password
- Enter your RSA token id
- (Only for a new profile name AND you have access to multiple accounts) Select the account you wish to access

- INFO: Successful access will generate temporary credentials into your .aws credentials file with a profile-named entry
- INFO: These temporary credentials expire after 8 hours - simply run again to regenerate new temp credentials
- INFO To set a particular profile name to be your default then set the environment variable: AWS_DEFAULT_PROFILE

STEP 3: Run your aws cli commands/Use the temporary access key credentials
- Example: Enter your aws cli commands - using --profile adfs (or set AWS_DEFAULT_PROFILE)
- Example: aws s3 ls --profile adfs  (if AWS_DEFAULT_PROFILE is not set)

STEP 4: For more help/options
- ./pcl aws --help
- ./pcl --help

INFO: If you are using an IDE, you will need to set your session to use the AWS_DEFAULT_PROFILE also
to ensure that you pick up the correct session token when running the code

Command:

./pcl aws --sandbox-user --domain asiapac --sid E900259

aws ec2 describe-instances --filters "Name=instance-state-name,Values=running" --query 'Reservations[].Instances[].[Tags[?Key==`Name`].Value, PublicIpAddress]'

