# TODO write a script to:
# TODO 1. clean table, i.e. drop everything before row 467 (as this is start of experiment video)
# TODO 2. check for the remaining instances if difference between two consecutive values is bigger than 5 or 10 or ...
# TODO    for all columns x y z yaw

import pandas as pd

df = pd.read_csv("locations_video.csv")
video_df = df[467:2963]
video_df = video_df.reset_index()
# print(video_df.head)
#print(video_df.iloc[0, 2:].tolist())

# check if difference between two consecutive location values have too big of a difference
count = 0
axis = ['x', 'y', 'z']
for i in range(0, len(video_df)-1, 20):
    # check if difference of 5 in x column
    loc1 = video_df.iloc[i, 2:-1]
    loc2 = video_df.iloc[i+1, 2:-1]
    for j in range(1):
        diff = abs(loc1[j]-loc2[j])
        if diff >= 20:
            count += 1
            print("Difference in location is too big: " + str(diff) + " in rows " + str(i) + " and " + str(i+1) +
                  " in axis:  " + str(axis[j]))
            print(loc1)
            print(loc2)
            print()
print(str(count) + " errors found; total rows: " + str(len(video_df)))
print("error percentage: " + str(count / len(video_df) * 100) + " % ")

# total row count: 2496

# x axis
# 211 errors for x axis for >20
# 168 errors for x axis for >30
# 148 errors for x axis for >40
# 131 errors for x axis for >50
# 131 errors for x axis for >50
# 108 errors for x axis for >60
# 98 errors for x axis for >70
# 75 errors for x axis for >100

# x axis with step 10 and >20: 20 errors
# x axis with step 15 and >20: 15 errors
# x axis with step 20 and >20: 11 errors


# y axis
# 203 errors for y axis for >20
# 146 errors for y axis for >30
# 106 errors for y axis for >40
# 82 errors for y axis for >50
# 74 errors for y axis for >60
# 64 errors for y axis for >70
# 47 errors for y axis for >100

# z axis
# 30 errors for y axis for >10
# 4 errors for y axis for >15
# 0 errors for y axis for >20
