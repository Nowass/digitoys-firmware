#!/bin/bash
# Helper script to export user/group IDs for docker-compose
# Usage: source .devcontainer/set-user-id.sh

export LOCAL_USER_ID=$(id -u)
export LOCAL_GROUP_ID=$(id -g)

echo "Set LOCAL_USER_ID=$LOCAL_USER_ID"
echo "Set LOCAL_GROUP_ID=$LOCAL_GROUP_ID"
echo ""
echo "You can now run: docker-compose up -d"
