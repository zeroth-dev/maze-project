# How to download and upload code to repository:

Download:
```
cd ~/<path_to_your_workspace>/src
git clone https://github.com/zeroth-dev/maze-project.git
git checkout -b <your_branch_name>
cd ..
catkin_make
```
Do the work on the code and for upload:
```
cd ~/<path_to_your_workspace>/src/maze-project
git add .
git commit
```
Write the changes done and save
```
git push origin <your_branch_name>
```
