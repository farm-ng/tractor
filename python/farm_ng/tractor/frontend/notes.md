2020.04.21 notes on frontend development
matthew.bitterman@gmail.com

FRONTEND DASHBOARD TUTORIAL
most of this dashboard was made following this tutorial :
https://www.youtube.com/watch?v=Ppz2UNlHp0g
code from video found here :
https://github.com/frontendfunn/bootstrap4-admin-dashboard


SCSS FILES
installed SASS following these instructions
(css alternative commonly used with bootstrap)
https://sass-lang.com/install
* installed using homebrew on my macos dev env
* would we want to download the package and just include it, adding to the path variable?

$ sass --version
>>> 1.26.3

usage:
it has to be run to compile a typical css file...
https://sass-lang.com/guide

$ sass --input.scss output.css
>>>

in order to get the .scss to automatically update the .css file :
$ sass --watch stylesheet.scss:stylesheet.css

looks like this is a process that needs to be run.  a little awkward there.

MATERIAL DESIGN ICONS
To see the icons we are instantiating, browse here.
https://material.io/resources/icons/?icon=cast_connected&style=baseline


COLUMNS in BOOTSTRAP
Utilizes Flexbox, good guide here.
https://css-tricks.com/snippets/css/a-guide-to-flexbox/#flexbox-background

Otherwise Bootstrap Documentation:
https://getbootstrap.com/docs/4.0/layout/grid/#auto-layout-columns
