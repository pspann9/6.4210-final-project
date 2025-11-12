## Open Project in docker Container Instructions

# Make sure that Docker is running.
# Open the project folder in VSCode.
# Press ⌘⇧P / Ctrl⇧P → “Dev Containers: Reopen in Container”.
# The first time you do this: Wait for the container to download and build, and for postCreateCommand to set up the virtual         environment inside the container (this will take ~15 minutes the first time you do this, but should be quick after)
# VSCode should auto-select ${workspaceFolder}/.venv/bin/python as the Python interpreter. If not, pick it in the interpreter menu.
# To close the Dev Container, select “File → Close Remote Connection”
