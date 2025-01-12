## ----------------------- watod Configuration File Override ----------------------------

##
## HINT: You can copy the contents of this file to a watod-config.local.sh 
##       file that is untrackable by git and readable by watod.
##

## ----------------------- watod Configuration File Override ----------------------------
## ACTIVE Modules CONFIGURATION
## List of active modules to run, defined in docker-compose.yaml.
##
## Possible values:
##   - vis_tools        :   starts tools for data visualization (foxglove)
##   - gazebo           :   starts robot simulator (gazebo)
##   - robot            :   starts up robot nodes
##   - samples          :   starts up sample nodes for reference

ACTIVE_MODULES="vis_tools gazebo robot"

################################# MODE OF OPERATION #################################
## Possible modes of operation when running watod.
## Possible values:
##	 - deploy (default)		:	runs production-grade containers (non-editable)
##	 - develop   		    :	runs developer containers (editable)

# MODE_OF_OPERATION=""

############################## ADVANCED CONFIGURATIONS ##############################
## Name to append to docker containers. DEFAULT = "<your_watcloud_username>"
# COMPOSE_PROJECT_NAME=""

## Tag to use. Images are formatted as <IMAGE_NAME>:<TAG> with forward slashes replaced with dashes.
## DEFAULT = "<your_current_github_branch>"
# TAG=""

# Docker Registry to pull/push images. DEFAULT = "ghcr.io/watonomous/wato_monorepo"
# REGISTRY_URL=""

## Platform in which to build the docker images with. 
## Either arm64 (apple silicon, raspberry pi) or amd64 (most computers)
# PLATFORM="amd64"
