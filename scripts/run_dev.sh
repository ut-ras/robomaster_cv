#!/bin/bash

SERVICE_NAME="robomaster_cv"

# Get the container ID from docker-compose
CONTAINER_ID=$(docker compose ps -q $SERVICE_NAME)

# Check if the container is running
if [ "$(docker ps -q -f id=$CONTAINER_ID)" ]; then
    echo "Container is already running. Connecting to it..."
else
    echo "Starting the container in daemon mode..."
    docker compose up -d

    # Wait for the container to start
    while [ -z "$(docker compose ps -q $SERVICE_NAME)" ]; do
        echo "Waiting for the container to start..."
        sleep 1
    done

    # Update the container ID after it starts
    CONTAINER_ID=$(docker compose ps -q $SERVICE_NAME)
fi

echo "Connecting to the container..."
MSYS_NO_PATHCONV=1 docker exec -u admin -it $CONTAINER_ID '/bin/bash'
