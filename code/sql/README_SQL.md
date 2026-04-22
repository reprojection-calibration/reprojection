# Principles

## Foreign key relationships

The primary foreign key relationship which structures the database is from the master `calibration_step` table to
everything else. There is should be essentially no way for data to enter the database except through a structure
calibration process step. Therefore, each row in each table should be assigned a `step_name` which describes from which
part of the calibration process the data originates from. As there are a known and finite number of calibration steps in
the process we put these into the master `calibration_step` table and all other tables have a foreign key relationship
on this table.

Letting business logic be so clearly represented in the database could potentially be a huge antipattern. However, my
personal experience developing this (my first time working with a database directly) showed me that foreign key
relationships are a powerful tool to enforce consistency. Time will tell if we need a different abstraction to help us
solve this problem.

Note that you must set `PRAGMA foreign_keys = ON;` for foreign keys to be respected! This is not enabled by default and
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