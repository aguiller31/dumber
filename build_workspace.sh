#!/bin/bash
echo "# Robot_Mobile" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M master
git remote add origin git@github.com:aguiller31/dumber.git
git push -u origin master
