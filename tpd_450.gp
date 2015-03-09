#
# Two ways of generating a 2D heat map from ascii data
#

set title "Heat Map generated from a file containing Z values only"
unset key
set tic scale 0

# Color runs from white to green
#set palette model RGB
#set palette defined 
set palette rgbformula -7,2,-7
set cbrange [0:609980]
set cblabel "Visit Count"
unset cbtics

set xrange [-0.5:6.5]
set yrange [-0.5:6.5]

set view map
splot '-' matrix with image 
 35419  156868  261628  297902  221653  191136   55724 
 142254  310597  383870  407962  336379  364558  139894 
 314419  420742  471181  506597  403861  491010  285901 
 369011  407486  481387  471708  401896  534388  303760 
 251545  328947  407213  609980  528934  495259  208004 
 191997  381068  451552  464521  414056  311592   92153 
  99343  163541  218111  219290  152496  101452   50113
e
e
