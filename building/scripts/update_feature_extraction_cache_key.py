import sqlite3, sys

# NOTE(Jack): If you are here and confused or thinking that whoever wrote this is doing some really hack stuff... you
# are in the right place. This script writes a feature extraction calibration step for the /cam0/image_raw sensor given
# a cache key from the user. Why do we need this?
#
# We want to be able to run the entire calibration pipeline in the docker worklfow and github actions CI. The only
# problem is that the test data we have checked into the repository is the dataset-calib-imu4_512_16.db3 database which
# only contains the extracted features from the TUM dataset-calib-imu4 dataset. There are no images in the database
# because they would take up too much space, and even if there were images they are of the custom Kalibr compatible
# aprilgrid target which as of now (9.4.2026), we do not support. That means that we could not run the feature
# extraction even if we wanted to in the integration tests.
#
# Therefore we need to artificially triger a cache hit for the feature extraction, which we do here by manually writing
# the dataset specific (i.e. ROS1 vs ROS2) cache key manually into the database. This means that the feature extraction
# is triggered as a cache hit ang the extracted feature are loaded directly from the database.
#
# This is extremely brittle. What if image feature cache key/signature calculation in the application changes? Then we
# will get a different cache key and need to replace the value passed to this script.
#
# But I think the real problem is a structural one, and that is that the cache key used for the feature extraction does
# not come from the images table in the database, but comes instead from bespoke application side user logic that each
# person/application writes themselves. What we should instead do is have the user write the images to the images table
# and then kick off the process from that point.
#
# At least that was my original intent before I got into the weeds here. The reason why I have not done that is because
# given the current architecture that would require serializing/deserializing the image three time. Once to load it
# from the application code and write into into the database in a consistent data format, once to write it to the images
# tables and then another read to load it for the feature extraction.
#
# And while I never benchmarked this, I am quite sure that a user with 1000 4k images will have to wait a long time to
# let this process run on anything but the best computers. The fact that we are having this conversation means that we
# clearly have some thinking and designing to do, and one day I hope the result of that will be the removal of this
# script and the hack it carries out. It could be that I have over thought the problems and therefore missed a simple
# solution, time will tell :)

db_path, new_cache = sys.argv[1], sys.argv[2]

conn = sqlite3.connect(db_path)
conn.execute(
    """
    INSERT INTO calibration_steps (step_name, sensor_name, cache_key)
    VALUES (?, ?, ?)
    """,
    ('feature_extraction', '/cam0/image_raw', new_cache)
)
conn.commit()
conn.close()