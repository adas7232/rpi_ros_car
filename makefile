# make git m="your message"
git:
	git add --ignore-errors .
	git commit -m "$m"
	git push -u origin master 