#!/bin/bash
# Yes, this script can be improved immensly...
cd ..

result=0

echo "No subjects are of invalid size - great!<br>" > too_long_subjects.txt
echo "No subjects with leading lower case characters - great!<br>" > leading_lowercases.txt
echo "No subjects with trailing periods - great!<br>" > trailing_periods.txt
echo "No body lines are too wide - great!<br>" > too_long_body_lines.txt
echo "No keywords are missing in keywords.txt - great!<br>" > missing_keywords.txt
echo "No keywords in code that don't exist in keywords.txt - great!<br>" > missing_keywords_2.txt
echo "No keywords in code that don't have Doxygen comments and aren't blacklisted in keywords.txt - great!<br>" > missing_keywords_3.txt
echo "No lines in keywords.txt using spaces instead of TAB (the Arduino IDE doesn't support space) - great!<br>" > tab_spaces_keywords.txt
echo "No occurences of the deprecated boolean data type - great!<br>" >> booleans.txt

too_long_subjects=`awk 'length > 72' subjects.txt`
if [ -n "$too_long_subjects" ]; then
  echo "<b>Commit subjects that are too wide (&gt;72 characters):</b>" > too_long_subjects.txt
  echo "$too_long_subjects" >> too_long_subjects.txt
  sed -i -e 's/$/<br>/' too_long_subjects.txt
  result=1
fi
leading_lowercases=`awk '/^[[:lower:][:punct:]]/' subjects.txt`
if [ -n "$leading_lowercases" ]; then
  echo "<b>Commit subjects with leading lowercase characters:</b>" > leading_lowercases.txt
  echo "$leading_lowercases" >> leading_lowercases.txt
  sed -i -e 's/$/<br>/' leading_lowercases.txt
  result=1
fi
trailing_periods=`awk '/(\.)$/' subjects.txt`
if [ -n "$trailing_periods" ]; then
  echo "<b>Commit subjects with trailing periods:</b>" > trailing_periods.txt
  echo "$trailing_periods" >> trailing_periods.txt
  sed -i -e 's/$/<br>/' trailing_periods.txt
  result=1
fi

too_long_body_lines=`awk 'length > 72' bodies.txt`
if [ -n "$too_long_body_lines" ]; then
  echo "<b>Body lines that are too wide (&gt;72 characters):</b>" > too_long_body_lines.txt
  echo "$too_long_body_lines" >> too_long_body_lines.txt
  sed -i -e 's/$/<br>/' too_long_body_lines.txt
  result=1
fi

missing_keywords=$(for keyword in $(grep -A999 '#if DOXYGEN' MyConfig.h | grep -B999 '#endif' | grep '#define' | awk '{ print $2 '} | grep -e '^MY_'); do grep -q $keyword keywords.txt || echo $keyword; done)
if [ -n "$missing_keywords" ]; then
  echo "<b>Keywords that are missing from keywords.txt:</b>" > missing_keywords.txt
  echo "$missing_keywords" >> missing_keywords.txt
  sed -i -e 's/$/<br>/' missing_keywords.txt
  result=1
fi

missing_keywords_2=$(SOURCE_FILES="core/ drivers/ hal/ examples/ examples_linux/ MyConfig.h MySensors.h"; for keyword in $(grep -whore  'MY_[A-Z][A-Z_0-9]*' $SOURCE_FILES | sort -u ); do grep -q $keyword keywords.txt || echo $keyword; done)
if [ -n "$missing_keywords_2" ]; then
  echo "<b>Keywords in code that don't exist in keywords.txt:</b>" > missing_keywords_2.txt
  echo "If keywords aren't in keywords.txt, they will not be highlighted in the Arduino IDE. Highlighting makes the code easier to follow and helps spot spelling mistakes." > missing_keywords_2.txt
  echo "$missing_keywords_2" >> missing_keywords_2.txt
  sed -i -e 's/$/<br>/' missing_keywords_2.txt
  result=1
fi

missing_keywords_3=$(SOURCE_FILES="core/ drivers/ hal/ examples/ examples_linux/ MyConfig.h MySensors.h"; for keyword in $(grep -whore  'MY_[A-Z][A-Z_0-9]*' $SOURCE_FILES | sort -u ); do grep -q $keyword keywords.txt || echo $keyword; done)
if [ -n "$missing_keywords_3" ]; then
  echo "<b>Keywords in code that don't have Doxygen comments and aren't blacklisted in keywords.txt:</b>" > missing_keywords_3.txt
  echo "If keywords don't have Doxygen comments, they will not be available at https://www.mysensors.org/apidocs/index.html Add Doxygen comments to make it easier for users to find and understand how to use the new keywords." > missing_keywords_3.txt
  echo "$missing_keywords_3" >> missing_keywords_3.txt
  sed -i -e 's/$/<br>/' missing_keywords_3.txt
  result=1
fi


tab_spaces_keywords=$(grep -e '[[:space:]]KEYWORD' -e '[[:space:]]LITERAL1' keywords.txt | grep -v -e $'\tLITERAL1' -e $'\tKEYWORD')
if [ -n "$tab_spaces_keywords" ]; then
  echo "<b>Keywords that use space instead of TAB in keywords.txt:</b>" > tab_spaces_keywords.txt
  echo "$tab_spaces_keywords" >> tab_spaces_keywords.txt
  sed -i -e 's/$/<br>/' tab_spaces_keywords.txt
  result=1
fi

# Evaluate if there exists booleans in the code tree (not counting this file)
if git grep -q boolean -- `git ls-files | grep -v butler.sh`; then
  echo "<b>You have added at least one occurence of the deprecated boolean data type. Please use bool instead.</b><br>" > booleans.txt
  result=1
fi

printf "%s" "<html>" > butler.html
echo "Greetings! Here is my evaluation of your pull request:<br>" >> butler.html
awk 'FNR==1{print "<br>"}1' too_long_subjects.txt leading_lowercases.txt trailing_periods.txt too_long_body_lines.txt missing_keywords.txt missing_keywords_2.txt missing_keywords_3.txt tab_spaces_keywords.txt booleans.txt >> butler.html
echo "<br>" >> butler.html
if [ $result -ne 0 ]; then
	echo "<b>I am afraid there are some issues with your commit messages and/or use of keywords.</b><br>" >> butler.html
	echo "I highly recommend reading <a href="http://chris.beams.io/posts/git-commit">this guide</a> for tips on how to write a good commit message.<br>" >> butler.html
	echo "More specifically, MySensors have some <a href="https://www.mysensors.org/download/contributing">code contribution guidelines</a> that I am afraid all contributers need to follow.<br>" >> butler.html
	echo "<br>" >> butler.html
	echo "I can help guide you in how to change the commit message for a single-commit pull request:<br>" >> butler.html
	echo "git checkout &lt;your_branch&gt;<br>" >> butler.html
	echo "git commit --amend<br>" >> butler.html
	echo "git push origin HEAD:&lt;your_branch_on_github&gt; -f<br>" >> butler.html
	echo "<br>" >> butler.html
	echo "To change the commit messages for a multiple-commit pull request:<br>" >> butler.html
	echo "git checkout &lt;your_branch&gt;<br>" >> butler.html
	echo "git rebase -i &lt;sha_of_parent_to_the_earliest_commit_you_want_to_change&gt;<br>" >> butler.html
	echo "Replace \"pick\" with \"r\" or \"reword\" on the commits you need to change message for<br>" >> butler.html
	echo "git push origin HEAD:&lt;your_branch_on_github&gt; -f<br>" >> butler.html
	echo "<br>" >> butler.html
fi

# Evaluate coding style
astyle --options=.mystools/astyle/config/style.cfg -nq --recursive "*.h" "*.c" "*.cpp"
git diff > restyling.patch
if [ -s restyling.patch ]; then
	echo "I am afraid your coding style is not entirely in line with the MySensors prefered style.<b><br>A mail with a patch has been sent to you that, if applied to your PR, will make it follow the MySensors coding standards.</b><br>" >> butler.html
	echo "You can apply the patch using:<br>" >> butler.html
	echo "git apply restyling.patch<br>" >> butler.html
	echo "<br>" >> butler.html
	result=1
else
	echo "This commit is meeting the coding standards, well done!<br>" >> butler.html
	echo "<br>" >> butler.html
	rm restyling.patch
fi

if [ $result -ne 0 ]; then
	echo "If you have any questions, please first read the <a href="https://www.mysensors.org/download/contributing">code contribution guidelines</a>.</b><br>" >> butler.html
	echo "<b>If you disagree to this, please discuss it in the GitHub pull request thread.</b><br>" >> butler.html
	echo "<br>" >> butler.html
fi
echo "Yours sincerely, The Butler, serving the MySensors community<br>" >> butler.html
printf "%s" "</html>" >> butler.html
exit $result
