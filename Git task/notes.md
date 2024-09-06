# Git and Github

## Some basic commands:

Personal Access Token: Use it as password for accessing repos 

**git config --global user.name "Your Name"**
**git config --global user.email "your.email@example.com"** 
These should be set before working on any project. Ideally, keep it same as GitHub username and email. In case of two GitHub accounts, they can be changed later at any point of time using the same commands.

When we are inside some folder(using cd), then the command **'git init'** initializes a git repository for that folder. This will track all the changes and commits to that specific directory.
**'ls .git'** -> gives HEAD, config, refs, etc. Otherwise only 'ls' won't give anything.

**'touch file_name.txt'** -> makes a new file

**'code file_name'** -> This will open the file in VS code if while installing Git Bash, VS code was selected as the default editor.

**'git status'** -> gives changes made to the directory inside which we initiated git repository

To add the changes to the staging area, type the command **'git add .'** This will stage all the changes made to the repository that are untracked, that is the directory.

If **'git add new.txt'**, then it stages only new.txt file.

Command **'git commit -m "Type here the message to be displayed"'** used for commiting these changes.

**'git restore --staged file_name.txt'** -> This will unstage the chnages that were staged.

**'git log'** -> gives a log file containing history of all the changes/commits

commits are stacked over one another. So newer commits get added to the top and each commit has its own id. If I want to remove all commits (remove means unstage them) above a commit with particular id, then I do **'git reset commit_id'**

**'git stash'** -> We have staged some changes but don't want to commit them, but rather save them (suh that we can access it again) somewhere and keep the working tree clean. It's like *"Hey you go to backstage, I will call you when needed"* (But remember, changes may or may not be in staging area before using this. So do "git add ." to add them to staging area before git stash.)

**'git stash pop'** -> Getting back those saved changes (not commits). It's like *"Hey those changes in the backstage area, come back to the staging area, I might need to commit you :)"*

**git stash clear** -> Remove those saved changes from the backstage (Delete them from memory. Not recoverable)

**git clean -f** -> To delete untracked files from the memory

## Git and GitHub:

First go to GitHub and create new repository.

**git remote add origin url_of_the_github_repo** -> This will create a remote repository out of the URL and then we can track changes using Git. So, when you execute **git remote add origin <remote_repository_URL>**, you're essentially telling Git to remember the URL of another repository (the remote repository) under the name *"origin"*. This allows you to interact with that remote repository using the shorthand "origin" instead of typing out the full URL every time you want to push or pull changes.

*Note:* -> The word *"remote"* means we are working with URLs. In above command, we are using origin as an alias for the whole URL.

**git push origin main** -> To share the changes (to be specific; commits) that were made in the local directory. Also used for pushing new commits to the main branch. Origin means we are the owner.
Without this, the commits will be made in local clone only.

**git push origin branch_name** -> To commit in any other branch associated with the same Git repository.

*Note:* -> To rewrite the branch history, like when we remove some commits or squash them, we need to force push the changes. So for that use **'git push origin branch_name -f'**

While squashing my commit with someone else's while sending a PR, it might be that the author name is previous author's. So to change the author's name in last(recent) commit, do **git commit --amend --author="Your New Name <your.email@example.com>"**. This opens in an editor. Then write your name and save and close. Again it opens in an editor. Then don't change the commit message and save and close.
It will work fine :) 

**git branch branch_name** -> Create a new branch (It is always created from the head). The default branch is main/master. There will be a '*' symbol next to the branch in which we are currently there

*Note:* -> Git Bash still shows the default name of branch as master. To rename local branch, write command **'git branch -m master main'** and **'git push -u origin main'** to push the branch to the remote repository.

**git checkout branch_name** -> Now our HEAD will point to the branch 'branch_name'. So there will be an asterisk beside its name. This means now whatever commits we do, they will be in this branch.

**git merge branch_name** -> This will merge the history of any branches other than main with the main branch. 

## Contribution to projects:

First fork the repository where we want to contribute.
Then clone it to the local machine.

**git clone url_of_the_forked_repo** -> Creates a clone in the local machine.

**git remote add upstream url_of_the_original_github_repo_from_which_we_forked** -> This will keep URL of the original repository under the name *"upstream"*.

Basically, we will be referring the forked repo in our account as **"origin"** and original repo from where we forked as **"upstream"**.

**git remote -v** -> Used to view the list of remote repositories associated with your local Git repository, along with their corresponding URLs. Here's what each part of the command does:

* git: This invokes the Git command-line tool.
* remote: This specifies that you want to work with remote repositories.
* -v (or --verbose): This option stands for "verbose" and it instructs Git to display more detailed information about each remote repository.

=> After cloning the forked repo, let's say we do some changes in the code and commit this in a new branch named 'feature'. Now we write 'git push origin feature' and this will push the changes to the forked repo. After this, we can make a pull request.
**Pitfall:** Only one pull request can be made per branch. If we commit anything else in the same branch, then it will be included in the same pull request only. So good practice is to create different branches for different features that we're working upon.

If I remove some commit from the pull request, and add do git stash, then I need to force push these changes. The command for that is **'git push origin working_branch_name -f'**

### Keeping main branch of the forked repo up to date (even) with the main branch of upstream:

Three ways:

1. Go to main branch of forked repo and after owner of upstream repo has merged the PR, you will see an option "Fetch Upstream" in the main branch of fork. Click it to make fork even with the upstream. 
1. Manually: First do git checkout main. Then fetch all the changes using **'git fetch --all --prune'**. After this, reset the main branch of the origin to the main branch of the upstream using **'git reset --hard upstream/main'**. Finally push these commits. Do **'git push origin main'**
1. Using git pull: Do **'git pull upstream main'** to pull the changes from the upstream URL's main branch. Then do **'git push origin main'** to push the changes in the fork.

-> Squash, is a feature within Git that allows you to condense multiple commits into a single commit. When you squash commits, you're combining them into one larger commit, effectively cleaning up your commit history and making it easier to understand. This is particularly useful before merging branches or before pushing your changes to a shared repository, as it helps keep the commit history clean and concise.

For squashing commits, do **git rebase -i <hash_of_commit_above_which_all_commits_to_be_squashed>**
Then write **'s'** in front of that commit which is to be merged into the pick commit. **'pick'** in front of that commit in which other commits will get merged.

Managing the commit history and keeping it up to date: 
1. **'git rebase'** -> It changes the base of the commits of new branch to the latest commit of the main branch and thus tries to maintain a linear commit history. 
