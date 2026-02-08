GIT Help (Basic Use guide)
===================================
Git is a version control system for tracking changes in files and coordinating work among multiple people. It is primarily used for source code management in software development and is the *de‑facto* standard for open‑source development. In this subject, all Git repositories are hosted on GitHub, which provides you with an **individual, private remote repository**.

[A video guide to be used in conjunction with this information is available on canvas](https://canvas.uts.edu.au/courses/38589/pages/github-and-cloning-your-repository)

Conventions used in this guide

- Terminal commands appear `like this`.
- Angle brackets indicate values you must replace. 
  - For example:`<username>` → *your GitHub username*
  - Do not type the `< >` characters.

## Your GitHub repository

Once you accept the GitHub Classroom invite using your GitHub account, GitHub automatically creates a new private repository just for you called  `pfms-2026a-<username>`.

This repository is created from a subject template, but it is independent of all other students’ repositories. You can manage your repository via the GitHub web interface at:`https://github.com/41012/pfms-2026a-<username>`. 

At any point in time, your local repository (on your computer) and your remote repository (on GitHub) may be different. The sections below explain how to keep them in sync.

Setting up the repository on your workstation
------------------------------------
This is a one-time process for your workstation (Azure/Your computer). Repeat the process if you use a different workstation.

We strongly recommend using SSH keys with GitHub. This allows you to authenticate securely without entering your password every time you use Git from the terminal.

Follow these three steps (only once):

1. [Generate a new SSH key](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent#generating-a-new-ssh-key)
2. [Adding your SSH key to the ssh-agent](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent#adding-your-ssh-key-to-the-ssh-agent)

3. [Add new SSH key to your GitHub Account](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)

## Clone your repository

Chose a location on your file system clone the repository (for example mine is under a subfolder `~/git` where I keep all my git repositories). 

On the GitHub website for your repository:

1. Click the green Code button
2. Select SSH (not HTTPS)
3. Copy the URL that starts with `git@github.com:`

Then on your workstation in the terminal run:

```
git clone git@github.com:41012/pfms-2026a-<username>.git
```
This creates a folder pfms-2026a-<username> - this is your local repository. 

**Note:** If cloning fails with a permissions error, your SSH key is not configured correctly.

## Configuring your Git identity

If this is your first time using Git on your workstation, set your name and email and state the `rebase` preference.

```
git config --global user.name "<your_name>"
git config --global user.email "<your_email>"
git config --global pull.rebase false 
```
These values label your commits. The email should match one registered with your GitHub account.

For instance ``git config --global user.name "John Smith"`` and ``git config --global user.email "John.Smith@email.com"``

Obtaining tutorial material
------------------------------------
We will make an announcement on [TEAMS] when material is available. From any folder inside your local repository, run:
```
git fetch origin subject
git merge origin/subject --allow-unrelated-histories
git push
```
- Tutorial material is distributed via a separate branch called `subject`.
- Only run these commands once per announcement.

The above will work seamlessly, as long as you have not created files/folders with the same name as those we are distributing. 
The merging happens on the level of your local repository and the last command `git push` pushes the merged results back to the remote repository, so the result is able to be seen on git.

If you see merge errors, post a message on [TEAMS] (Software channel).


Obtaining quiz material and feedback
------------------------------------
We will make an announcement on [TEAMS] when material is available. From any folder inside your local repository, run:
```
git pull
```
**Strong recommendation: ``git pull`` every time you start working on your repository**

Managing your own files
------------------------------------
Check the status of your repository:
```
git status 
```
If you wish to stage files for commit:
```
git add <file_or_folder> 
```
To commit files to your local repo, and specify a `<message>` which describes the nature of the changes you have made use:
```
git commit -m “<message>”
```
It is important to use descriptive commit messages so that later you can make sense of your commits using `git log`.

**NOTE:** If you do not specify a message, you will be prompted for one, they can not be blank. A terminal-based text editor will appear to write a message, after writing it hit CTRL+X, then Y, then ENTER. 

To publish your local commits to your hosted repository use:
```
git push
```

You can find **much** more info about git online. Be aware that [git](https://git-scm.com) is not the same as [github](https://github.com). git is a version control system. github is just a popular website that provides hosted git repositories.

Git Workflow Explained
------------------------------------

Git is not cloud storage (ie. DropBox/Google Drive), it is designed for managing code. Managing code well would imply that their is a version of your code which compiles / does not break other dependent code, that code is stable and available to you (and your team if working in a team). 

Obviously, cloud storage has no understanding of this requirement, and simply updates your storage with any changes, as it was designed for documents/text/images. Further, cloud storage has no knowledge of which files need to be backed up, programming has many intermediate files that are simply a by-product of build/link process and specific to your computer. 

Therefore, the onus of managing your code lies on the developer, *you decide* when to *push* the code to hosted/remote repository (from here on repository is referred to as *repo*). 

To allow users to nominate which code to store in version control, git has a staging process. You must therefore notify git which code to consider using the ``git add`` command.

To achieve the set objectives of versioning code git has a few layers, it considers a local and a remote repo, that is, git allows you to locally manage your code as well, work on it and then push it to the remote. In this way there is versioning on your local repo for code not ready to be pushed to remote repo. Therefore we have ``git commit`` to achieve version control on your local repo, and ``git push`` to push it to the remote repo. Until the push your code does not exist on remote repo (therefore is not backed-up/in-the-cloud so to speak). 

To obtain code from remote repo you use ``git pull`` which is actually a two phase process ``git fetch`` followed by a ``git merge``.

An illustration of this follows- image from https://tex.stackexchange.com/questions/70320/workflow-diagram)
![alt text](https://i.stack.imgur.com/5V7uJ.png "Git workflow")


------------------------------------

## Final advice

Do not panic if you make a mistake. 

Git is designed to recover from errors. If something looks wrong, stop and ask our lab tutors or on [TEAMS] before trying random commands.



[TEAMS]: https://teams.microsoft.com/l/team/19%3AoAl1-epPNBfAorXHC9R4YEkJPnXEhnqPtY97Q4zUPis1%40thread.tacv2/conversations?groupId=0849c4da-7f32-4939-b1d1-97d0dd93a691&tenantId=e8911c26-cf9f-4a9c-878e-527807be8791

