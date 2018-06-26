Direct Transcription Co-Design (DTCD) Toolbox

= Description =

 This toolbox consists of a set of MATLAB files
that implement a few different
algorithms for solving co-design problems using direct transcription. MDO
problem
consists of a design optimization sub-problem and a control-design 
sub-problem. 

See run_comparison.m for the
top-level file that demonstrates the use of this toolbox using the wing problem described in the article:
'Integrated structural and control system design(Co-Design) for robust flutter performance'. 

Author: Giovanni Filippi, ISAE-SUPAERO Student
To plot results use the function myplot_results('platform')
In MATLAB command window, use

  >> addpath(fullfile(pwd, 'solver'))



Requirements:

 - MATLAB 8.0 and later
 - Optimization toolbox version 6.2 and later
 - MATLAB xUnit test framework
