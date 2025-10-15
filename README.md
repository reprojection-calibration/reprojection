# reprojection

## Github Actions

I would have liked to have all the actions (ex. [build](.github/workflows/build.yml) etc.) in one workflow file,
ordered like (1) code checks (2) build (3) code coverage assertion. However, the only way that I could actually get the
docker cache (needed to keep build times down) to work in between executions and not just within one pipeline execution
was to put them in separate workflow files. This means that they do not execute in a specific order, but at least the
cache works! within each pipeline. 

