# Principles

## Foreign key relationships

"A foreign key is a set of attributes in a table that refers to the primary key of another table, linking these two
tables." - https://en.wikipedia.org/wiki/Foreign_key

We use foreign keys to encode the dependency of data on stages and other data in the calibration process. There are
three common foreign key semantics in the calibration database. A single "step" foreign key dependency, a dual "step and
data" dependency, or a single "data" dependency. An example of a "single" step dependency is found in the
`camera_info_table`:

    FOREIGN KEY (step_name, sensor_name) REFERENCES calibration_steps ON DELETE CASCADE

An example of a dual "step and data" dependency is found in the `camera_intrinsics` table:

    FOREIGN KEY (step_name, sensor_name) REFERENCES calibration_steps ON DELETE CASCADE,
    FOREIGN KEY (sensor_name, camera_model) REFERENCES camera_info ON DELETE CASCADE

An example of a single data dependency is found in the `reprojection_error` table:

    FOREIGN KEY (step_name, sensor_name, timestamp_ns) REFERENCES poses ON DELETE CASCADE

Please only use the minumum possible foreign key relationship to represent a dependency. For example, the
`reprojection_error` in some sense logically also depends on the step in the calibration process that it was created in.
However, there is a 1-1 relationship between poses and reprojection errors and the step is already encoded in the pose
table. Just like in regular programming, if you find yourself duplicating information than it's is a sign you might be
missing the point.

Warning - Foreign key constraints are used to represent calibration process business logic in the database. This might
be an antipattern to code the calibration logic directly into the database, but my experience has shown that foreign key
relationships keep the database safe.

Note - You must set `PRAGMA foreign_keys = ON;` for foreign keys to be respected! This is not enabled by default and
might cause the database to not behave as expected if you are manipulating it in third party tools like CLion.

## Cascading delete

If a step is deleted then all data with a foreign key relationship on that step should also be deleted. This is critical
to keep the database clean when we need to rerun a step and remove the old data and write in the new data. Fundamentally
the caching logic implemented in `step_runner.hpp` is responsible for orchestrating this.

Establishing cascading delete relationships using `ON DELETE CASCADE` in the sql table directly prevents us from having
to manually delete all dependent data. That would be error-prone and harder to maintain in the c++ source code than it
is to code directly into the sql table definitions.

# Notes and disclaimers

## The camera info table (22.04.2026)

In the calibration process the camera info step is unique in that it does not actually calculate anything. Instead, it
basically just collects a set of metadata which defines the camera being calibrated. This happens at the very start of
the calibration process and is a pretty cut and dry process.

It is so simple in fact that I at first decided there was no need to include the `step_name` in the table. Why would we
need to specify the source step when there is only one possible step that it could have come from? That is in contrast
to things like a pose which might come from an initialization step and then again from an optimization step and
therefore require the step name.

The reason that we need the `step_name`, even for the camera info table, is that we need to establish a foreign key
relationship with the `calibration_steps` table. In order to do that we need to reference the steps table composite
primary key `PRIMARY KEY (step_name, sensor_name)` which is what forces us to include `step_name`. Without that column
in our `camera_info` table we cannot establish the foreign key relationship and take advantage of the cascading delete
semantics.

Camera info can only come from the `camera_info` step so I added the `CHECK ( step_name IN ('camera_info'))` constraint
to guarantee this at the database level. Again this is a pretty major case of business logic being coded into the
database and I might be shooting myself in the foot... only time will tell :)

# Brainstorming

## Store the image data in a video file

Storing the images in the database as individually encoded images takes up a ton of space and also CPU to get them there
(i.e. to encode/decode them). It could also be an idea to store a video file (ex. mp4 etc.) in a table as a blob and
reference that instead. What I have noticed is that 100mb videos recorded will turn into 2GB bag files and
correspondingly larger database files.

At this point I am a little afraid to dig into the image/video/codec/bitrate/formatting jungle, but clearly the current
status quo (22.04.20266) is not maintainable for the long term.