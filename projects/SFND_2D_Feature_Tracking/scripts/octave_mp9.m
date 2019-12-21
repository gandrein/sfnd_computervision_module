
% SHI_TOMASI detector

brisk_avg_totalTime  = mean(table2array(resultsSHITOMASIBRISKsummary(:,9)));
brief_avg_totalTime  = mean(table2array(resultsSHITOMASIBRIEFsummary(:,9)));
freak_avg_totalTime  = mean(table2array(resultsSHITOMASIFREAKsummary(:,9)));
orb_avg_totalTime    = mean(table2array(resultsSHITOMASIORBsummary(:,9)));
sift_avg_totalTime   = mean(table2array(resultsSHITOMASISIFTsummary(:,9)));

[brisk_avg_totalTime, 0, brief_avg_totalTime, freak_avg_totalTime, orb_avg_totalTime, sift_avg_totalTime]'


% HARRIS detector
brisk_avg_totalTime  = mean(table2array(resultsHARRISBRISKsummary(:,9)));
brief_avg_totalTime  = mean(table2array(resultsHARRISBRIEFsummary(:,9)));
freak_avg_totalTime  = mean(table2array(resultsHARRISFREAKsummary(:,9)));
orb_avg_totalTime    = mean(table2array(resultsHARRISORBsummary(:,9)));
sift_avg_totalTime   = mean(table2array(resultsHARRISSIFTsummary(:,9)));
[brisk_avg_totalTime, 0, brief_avg_totalTime, freak_avg_totalTime, orb_avg_totalTime, sift_avg_totalTime]'


% AKAZE detector

akaze_avg_totalTime  = mean(table2array(resultsAKAZEAKAZEsummary(:,9)));
brisk_avg_totalTime  = mean(table2array(resultsAKAZEBRISKsummary(:,9)));
brief_avg_totalTime  = mean(table2array(resultsAKAZEBRIEFsummary(:,9)));
freak_avg_totalTime  = mean(table2array(resultsAKAZEFREAKsummary(:,9)));
orb_avg_totalTime    = mean(table2array(resultsAKAZEORBsummary(:,9)));
sift_avg_totalTime   = mean(table2array(resultsAKAZESIFTsummary(:,9)));
[brisk_avg_totalTime, akaze_avg_totalTime, brief_avg_totalTime, freak_avg_totalTime, orb_avg_totalTime, sift_avg_totalTime]'


% BRISK detector
brisk_avg_totalTime  = mean(table2array(resultsBRISKBRISKsummary(:,9)));
brief_avg_totalTime  = mean(table2array(resultsBRISKBRIEFsummary(:,9)));
freak_avg_totalTime  = mean(table2array(resultsBRISKFREAKsummary(:,9)));
orb_avg_totalTime    = mean(table2array(resultsBRISKORBsummary(:,9)));
sift_avg_totalTime   = mean(table2array(resultsBRISKSIFTsummary(:,9)));
[brisk_avg_totalTime, 0, brief_avg_totalTime, freak_avg_totalTime, orb_avg_totalTime, sift_avg_totalTime]'


% FAST detector
brisk_avg_totalTime  = mean(table2array(resultsFASTBRISKsummary(:,9)));
brief_avg_totalTime  = mean(table2array(resultsFASTBRIEFsummary(:,9)));
freak_avg_totalTime  = mean(table2array(resultsFASTFREAKsummary(:,9)));
orb_avg_totalTime    = mean(table2array(resultsFASTORBsummary(:,9)));
sift_avg_totalTime   = mean(table2array(resultsFASTSIFTsummary(:,9)));
[brisk_avg_totalTime, 0, brief_avg_totalTime, freak_avg_totalTime, orb_avg_totalTime, sift_avg_totalTime]'



% ORB detector
brisk_avg_totalTime  = mean(table2array(resultsORBBRISKsummary(:,9)));
brief_avg_totalTime  = mean(table2array(resultsORBBRIEFsummary(:,9)));
freak_avg_totalTime  = mean(table2array(resultsORBFREAKsummary(:,9)));
orb_avg_totalTime    = mean(table2array(resultsORBORBsummary(:,9)));
sift_avg_totalTime   = mean(table2array(resultsORBSIFTsummary(:,9)));
[brisk_avg_totalTime, 0, brief_avg_totalTime, freak_avg_totalTime, orb_avg_totalTime, sift_avg_totalTime]'



% SIFT detector
brisk_avg_totalTime  = mean(table2array(resultsSIFTBRISKsummary(:,9)));
brief_avg_totalTime  = mean(table2array(resultsSIFTBRIEFsummary(:,9)));
freak_avg_totalTime  = mean(table2array(resultsSIFTFREAKsummary(:,9)));
sift_avg_totalTime   = mean(table2array(resultsSIFTSIFTsummary(:,9)));
[brisk_avg_totalTime, 0, brief_avg_totalTime, freak_avg_totalTime, 0, sift_avg_totalTime]'


