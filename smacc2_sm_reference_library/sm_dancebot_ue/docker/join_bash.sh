sudo docker exec -it $(sudo docker ps -aqf "ancestor=ue_editor_rclue:humble"| head -n 1) /bin/bash
