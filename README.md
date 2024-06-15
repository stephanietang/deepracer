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
| s11 | 180 | reward_function_speed.py | default | 38.899s | 7 | space action used in TIF, change DEEP to SHALLOW(default) |
| **s12 <br> s12-1 <br> s12-2** | 180<br>180<br>180 | reward_function_angle.py | default |  27.545s | 1 | action space used in TIF, stable model, average 75%, max 100%, min time during training is 22.581s, it is able to finish the laps without off track | 
| s13 | 180 | reward_function_angle2.py | default |  27.873s | 1 | |
| s14 | 180 | reward_function_minimalist.py | default |  36.953s | 5 | |
| s15 | 180 | reward_function_steps_progress.py | default |  46.743s | 10 | |
| s16 | 180 | reward_function_minimalist_speed.py | default |  59.930s | 17 | discarded, there is a bug in this reward function |
| s17 | 170 | reward_function_encouraging_racing_line.py | default |  30.327s | 1 |  |
| s18 | 480 | reward_function_optimal_trace2.py | default |   |  | use Capstone_AWS_DeepRacer to calculate the optimal paths and optimal space actions model_metadata_optimal.json use 2.5 as min speed, 4 as max speed. There is bug on it then discard this version  |
| s19 | 480 | reward_function_optimal_trace2.py | lr=0.0005 |   |  | There is bug on it then discard this version  |
| s20 | 480 | reward_function_optimal_trace2.py | lr=0.001<br>epocs=5<br>batch=32 |   |  | There is bug on it then discard this version |
| s21 | 480 | reward_function_optimal_trace2.py | default |   |  | There is bug on it then discard this version  |
| s22<br>s22-1 | 120<br>120 | reward_function_optimal_trace2.py | lr=0.0005 |   |  | remove steps_reward, add debugging log for first_racingpoint_index for each step <br>trained 4 hours average progress 12.7 max progress 43 |
| s23 | 120 | reward_function_optimal_trace2.py | default |   |  | remove steps_reward, add debugging log for first_racingpoint_index for each step <br>trained 4 hours average progress 13 max progress 46 |
| s24 | 120 | reward_function_optimal_trace2.py | lr=0.001<br>epochs=5<br>batch=32 |   |  | same as s23 trained 2 hours, average progress 2.72 max progress 22 too bad discarded |
| s25<br>s25-1 | 120<br>120 | reward_function_optimal_trace2.py | discount factor=0.5 |   |  | average progress 7.72 max progress 26 too bad discarded |
| s26 | 120 | reward_function_optimal_trace2.py | default |   |  | trained on top of s23, but the progress is still very small, average progress: 14 max progress: 49 |
| s27<br>s27-1 | 240<br>240 | TIF reward function | TIF hp |   |  | 37% 79% |
| s28<br>s28-1 | 240<br>240 | reward_function_speed.py | TIF hp |   |  | retrain s07 to see if the model performance is stable 40% 75% |
| s29 | 180 | reward_function_angle.py | default |   |  | use model_metadata_optimal_max_5_min_2.json |
| s30<br>s30-1<br>s30-2 | 120<br>120<br>480 | reward_function_progress_velocity.py | default |   |  | use model_metadata_7_AS.json failed at s30-2 too long time|
| s31 | 120 | my version of reward_function_progress_velocity.py | default |   |  | use model_metadata_optimal_max_4_min_2.json |



## How to Run the scripts
- Change the models config under ./models
- change every time in run.env
    - `DR_LOCAL_S3_MODEL_PREFIX`
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
- understand the parameters in reward function
    - https://github.com/dmh23/deep_racer_framework
- An advanced Guide to AWS DeepRacer https://towardsdatascience.com/an-advanced-guide-to-aws-deepracer-2b462c37eea
- AWS DeepRacer event for EPAM: my approach to taking the 4th place https://medium.com/@rostyslav.myronenko/aws-deepracer-event-for-epam-my-approach-for-taking-the-4th-place-ff3f76f39b1e
- How we broke into the top 1% of the AWS DeepRacer Virtual Circuit https://blog.gofynd.com/how-we-broke-into-the-top-1-of-the-aws-deepracer-virtual-circuit-573ba46c275
- Craft a Powerful Reward Function for AWS DeepRacer Student League https://medium.com/@anshml/

- github:
    - deepracer-on-the-sport https://github.com/aws-deepracer-community/deepracer-on-the-spot
    - tracks: https://github.com/aws-deepracer-community/deepracer-race-data/blob/main/raw_data/tracks/README.md
    - deepracer-analysis: https://github.com/aws-deepracer-community/deepracer-analysis
    - deepracer-log-guru https://github.com/aws-deepracer-community/deepracer-log-guru
        - go to log guru, run `python -m src.main.guru` 
    - calculate the optimal racelines: https://github.com/cdthompson/deepracer-k1999-race-lines
    - calculate the optimal speed and action space: https://github.com/dgnzlz/Capstone_AWS_DeepRacer