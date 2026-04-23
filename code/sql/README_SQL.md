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

Please only use the minimum possible foreign key relationship to represent a dependency. For example, the
`reprojection_error` in some sense logically also depends on the step in the calibration process that it was created in.
However, there is a 1-1 relationship between poses and reprojection errors and the step is already encoded in the pose
table. Just like in regular programming, if you find yourself duplicating information than it's is a sign you might be
missing the point.

Warning - Foreign key constraints are used to represent calibration process business logic in the database. This might
be an antipattern to code the calibration logic directly into the database, but my experience has shown that foreign key
relationships keep the database safe.

Note - You must set `PRAGMA foreign_keys = ON;` for foreign keys to be respected! This is not enabled by default and
might cause the database to not behave as expected if you are manipulating it in third party tools like CLion.

## Cascade delete

"Whenever rows in the parent (referenced) table are deleted (or updated), the respective rows of the child (referencing)
table with a matching foreign key column will be deleted (or updated) as well. This is called a cascade delete (or
update)." - https://en.wikipedia.org/wiki/Foreign_key#CASCADE

We use caching to store calculations from the calibration in the database. When that cache becomes invalidated (ex. you
change the target extraction config after extracting the targets) all the data in the invalidated step needs to be
removed. Foreign key relationships and the `ON DELETE CASCADE` enforces this.

# Notes and disclaimers

## Redundant `step_name` columns (22.04.2026)

The `camera_info` and `extracted_targets` tables both contain a `step_name` column. This is a piece of
duplicated/redundant information because by their very definition they are products of the camera info and feature
extraction steps respectively. Why do we do this then?

Including the `step_name` column lets us establish a foreign key relationship to the `calibration_steps` table. That
ensures that the application can apply a common interface to every step in the calibration process. If a step gets
removed then all dependent data (i.e. cache busted) gets removed to.

As a precaution, and to ensure that the `step_name` is really only ever the correct value for the `camera_info` and
`extracted_targets` tables I added a constraint to guarantee this at the database level using
`CHECK ( step_name IN ('camera_info'))`.

# Brainstorming

## Store the image data in a video file

Storing the images in the database as individually encoded images takes up a ton of space and also CPU to get them there
(i.e. to encode/decode them). It could also be an idea to store a video file (ex. mp4 etc.) in a table as a blob and
reference that instead. What I have noticed is that 100mb videos recorded will turn into 2GB bag files and
correspondingly larger database files.

At this point I am a little afraid to dig into the image/video/codec/bitrate/formatting jungle, but clearly the current
status quo (22.04.20266) is not maintainable for the long term.