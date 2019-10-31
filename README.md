# HK2019Skogforsk
## How to install

1. Make sure [ROS-melodic](http://wiki.ros.org/melodic/Installation) and catkin is installed, if not, follow the tutorials on the ROS website
2. Clone repository to where you want it with 
```bash
git clone https://github.com/Auto2/HK2019Skogforsk.git
```
3. Inside the folder HK2019Skogforsk open a terminal and write
```bash
catkin_make
```
4. Now it's set up!

## How to work with branches
1. If you are not up to date with master you can start with:
```bash
git pull origin master
```
2. When you are starting with a new issue/ feature etc. you'll have to create a new branch which is like a "alternative timeline"
```bash
git checkout -b "your_branch_name"
```
-b means you'll create a new branch, and "checkout" will change branch to the branch. If you want to change to another already existing branch you do that with:
```bash
git checkout "branch_name"
```

3. Use ```git status``` to get status on what branch you're on, what files has been modifed etc. 

4. When your code compiles and works as desired you will need to add the files. If you write ```git status```, files that are red are not added yet. Write ```git add .``` to add all the files or ```git add path/file_name``` to add a specific file. If you write ```git status``` once again you'll see that the added files now appears to be green instead.
5. Now it's time to commit the changes! 
```bash
git commit -m "message that describes your issue/ feature"
```
6. Now you're ready to push your work!
```bash
git push origin branch_name
```

If you want to merge your code to master you'll need to create a "pull request". This can be done at the github web page under "branches". Find you branch, create a pull request (you can see the changes that has been made here aswell). Another person in the project will have to review your pull request and accept it or decline. It is also possible to write comments, for example if anyone finds a bug/ has another solution.
