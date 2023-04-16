#!/bin/bash
CLIENT_IMAGE=${CLIENT_IMAGE:-test}
docker build . -t $CLIENT_IMAGE
# CLIENT_IMAGE=docker.discover-lab.com:55555/sztu/client:attack-0.0.3-ai bash scripts/build.sh
# CLIENT_IMAGE=docker.discover-lab.com:55555/sztu/client:defend-0.0.3-ai bash scripts/build.sh
