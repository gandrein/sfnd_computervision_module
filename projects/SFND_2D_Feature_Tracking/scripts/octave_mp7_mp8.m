
% SHI_TOMASI detector

avg_roi = 	    mean(table2array(resultsSHITOMASIBRISKsummary(1:end,4)));
avg_matched   = mean(table2array(resultsSHITOMASIBRISKsummary(2:end,5)));
avg_desrTime  = mean(table2array(resultsSHITOMASIBRISKsummary(:,7)));
avg_matchTime = mean(table2array(resultsSHITOMASIBRISKsummary(2:end,8)));

avg_roi = 	    mean(table2array(resultsSHITOMASIBRIEFsummary(1:end,4)));
avg_matched   = mean(table2array(resultsSHITOMASIBRIEFsummary(2:end,5)));
avg_desrTime  = mean(table2array(resultsSHITOMASIBRIEFsummary(:,7)));
avg_matchTime = mean(table2array(resultsSHITOMASIBRIEFsummary(2:end,8)));

avg_roi = 	    mean(table2array(resultsSHITOMASIFREAKsummary(1:end,4)));
avg_matched   = mean(table2array(resultsSHITOMASIFREAKsummary(2:end,5)));
avg_desrTime  = mean(table2array(resultsSHITOMASIFREAKsummary(:,7)));
avg_matchTime = mean(table2array(resultsSHITOMASIFREAKsummary(2:end,8)));

avg_roi = 	    mean(table2array(resultsSHITOMASIORBsummary(1:end,4)));
avg_matched   = mean(table2array(resultsSHITOMASIORBsummary(2:end,5)));
avg_desrTime  = mean(table2array(resultsSHITOMASIORBsummary(:,7)));
avg_matchTime = mean(table2array(resultsSHITOMASIORBsummary(2:end,8)));


avg_roi = 	    mean(table2array(resultsSHITOMASISIFTsummary(1:end,4)));
avg_matched   = mean(table2array(resultsSHITOMASISIFTsummary(2:end,5)));
avg_desrTime  = mean(table2array(resultsSHITOMASISIFTsummary(:,7)));
avg_matchTime = mean(table2array(resultsSHITOMASISIFTsummary(2:end,8)));


% HARRIS detector

avg_roi = 	    mean(table2array(resultsHARRISBRISKsummary(1:end,4)));
avg_matched   = mean(table2array(resultsHARRISBRISKsummary(2:end,5)));
pct = avg_matched/avg_roi*100
avg_desrTime  = mean(table2array(resultsHARRISBRISKsummary(:,7)));
avg_matchTime = mean(table2array(resultsHARRISBRISKsummary(2:end,8)));

avg_roi = 	    mean(table2array(resultsHARRISBRIEFsummary(1:end,4)));
avg_matched   = mean(table2array(resultsHARRISBRIEFsummary(2:end,5)));
pct = avg_matched/avg_roi*100
avg_desrTime  = mean(table2array(resultsHARRISBRIEFsummary(:,7)));
avg_matchTime = mean(table2array(resultsHARRISBRIEFsummary(2:end,8)));

avg_roi = 	    mean(table2array(resultsHARRISFREAKsummary(1:end,4)));
avg_matched   = mean(table2array(resultsHARRISFREAKsummary(2:end,5)));
pct = avg_matched/avg_roi*100
avg_desrTime  = mean(table2array(resultsHARRISFREAKsummary(:,7)));
avg_matchTime = mean(table2array(resultsHARRISFREAKsummary(2:end,8)));

avg_roi = 	    mean(table2array(resultsHARRISORBsummary(1:end,4)));
avg_matched   = mean(table2array(resultsHARRISORBsummary(2:end,5)));
pct = avg_matched/avg_roi*100
avg_desrTime  = mean(table2array(resultsHARRISORBsummary(:,7)));
avg_matchTime = mean(table2array(resultsHARRISORBsummary(2:end,8)));


avg_roi = 	    mean(table2array(resultsHARRISSIFTsummary(1:end,4)));
avg_matched   = mean(table2array(resultsHARRISSIFTsummary(2:end,5)));
pct = avg_matched/avg_roi*100
avg_desrTime  = mean(table2array(resultsHARRISSIFTsummary(:,7)));
avg_matchTime = mean(table2array(resultsHARRISSIFTsummary(2:end,8)));



% AKAZE detector

avg_roi = 	    mean(table2array(resultsAKAZEAKAZEsummary(1:end,4)));
avg_matched   = mean(table2array(resultsAKAZEAKAZEsummary(2:end,5)));
avg_desrTime  = mean(table2array(resultsAKAZEAKAZEsummary(:,7)));
avg_matchTime = mean(table2array(resultsAKAZEAKAZEsummary(2:end,8)));
pct = avg_matched/avg_roi*100

avg_roi = 	    mean(table2array(resultsAKAZEBRISKsummary(1:end,4)));
avg_matched   = mean(table2array(resultsAKAZEBRISKsummary(2:end,5)));
avg_desrTime  = mean(table2array(resultsAKAZEBRISKsummary(:,7)));
avg_matchTime = mean(table2array(resultsAKAZEBRISKsummary(2:end,8)));
pct = avg_matched/avg_roi*100

avg_roi = 	    mean(table2array(resultsAKAZEBRIEFsummary(1:end,4)));
avg_matched   = mean(table2array(resultsAKAZEBRIEFsummary(2:end,5)));
avg_desrTime  = mean(table2array(resultsAKAZEBRIEFsummary(:,7)));
avg_matchTime = mean(table2array(resultsAKAZEBRIEFsummary(2:end,8)));
pct = avg_matched/avg_roi*100

avg_roi = 	    mean(table2array(resultsAKAZEFREAKsummary(1:end,4)));
avg_matched   = mean(table2array(resultsAKAZEFREAKsummary(2:end,5)));
avg_desrTime  = mean(table2array(resultsAKAZEFREAKsummary(:,7)));
avg_matchTime = mean(table2array(resultsAKAZEFREAKsummary(2:end,8)));
pct = avg_matched/avg_roi*100

avg_roi = 	    mean(table2array(resultsAKAZEORBsummary(1:end,4)));
avg_matched   = mean(table2array(resultsAKAZEORBsummary(2:end,5)));
avg_desrTime  = mean(table2array(resultsAKAZEORBsummary(:,7)));
avg_matchTime = mean(table2array(resultsAKAZEORBsummary(2:end,8)));
pct = avg_matched/avg_roi*100

avg_roi = 	    mean(table2array(resultsAKAZESIFTsummary(1:end,4)));
avg_matched   = mean(table2array(resultsAKAZESIFTsummary(2:end,5)));
avg_desrTime  = mean(table2array(resultsAKAZESIFTsummary(:,7)));
avg_matchTime = mean(table2array(resultsAKAZESIFTsummary(2:end,8)));
pct = avg_matched/avg_roi*100


% BRISK detector

avg_roi = 	    mean(table2array(resultsBRISKBRISKsummary(1:end,4)));
avg_matched   = mean(table2array(resultsBRISKBRISKsummary(2:end,5)));
avg_desrTime  = mean(table2array(resultsBRISKBRISKsummary(:,7)));
avg_matchTime = mean(table2array(resultsBRISKBRISKsummary(2:end,8)));
pct = avg_matched/avg_roi*100

avg_roi = 	    mean(table2array(resultsBRISKBRIEFsummary(1:end,4)));
avg_matched   = mean(table2array(resultsBRISKBRIEFsummary(2:end,5)));
avg_desrTime  = mean(table2array(resultsBRISKBRIEFsummary(:,7)));
avg_matchTime = mean(table2array(resultsBRISKBRIEFsummary(2:end,8)));
pct = avg_matched/avg_roi*100

avg_roi = 	    mean(table2array(resultsBRISKFREAKsummary(1:end,4)));
avg_matched   = mean(table2array(resultsBRISKFREAKsummary(2:end,5)));
avg_desrTime  = mean(table2array(resultsBRISKFREAKsummary(:,7)));
avg_matchTime = mean(table2array(resultsBRISKFREAKsummary(2:end,8)));
pct = avg_matched/avg_roi*100

avg_roi = 	    mean(table2array(resultsBRISKORBsummary(1:end,4)));
avg_matched   = mean(table2array(resultsBRISKORBsummary(2:end,5)));
avg_desrTime  = mean(table2array(resultsBRISKORBsummary(:,7)));
avg_matchTime = mean(table2array(resultsBRISKORBsummary(2:end,8)));
pct = avg_matched/avg_roi*100

avg_roi = 	    mean(table2array(resultsBRISKSIFTsummary(1:end,4)));
avg_matched   = mean(table2array(resultsBRISKSIFTsummary(2:end,5)));
avg_desrTime  = mean(table2array(resultsBRISKSIFTsummary(:,7)));
avg_matchTime = mean(table2array(resultsBRISKSIFTsummary(2:end,8)));
pct = avg_matched/avg_roi*100

% FAST detector

brisk_avg_roi = 	  mean(table2array(resultsFASTBRISKsummary(1:end,4)));
brisk_avg_matched   = mean(table2array(resultsFASTBRISKsummary(2:end,5)));
brisk_avg_desrTime  = mean(table2array(resultsFASTBRISKsummary(:,7)));
brisk_avg_matchTime = mean(table2array(resultsFASTBRISKsummary(2:end,8)));
brisk_pct = brisk_avg_matched/brisk_avg_roi*100

brief_avg_roi = 	  mean(table2array(resultsFASTBRIEFsummary(1:end,4)));
brief_avg_matched   = mean(table2array(resultsFASTBRIEFsummary(2:end,5)));
brief_avg_desrTime  = mean(table2array(resultsFASTBRIEFsummary(:,7)));
brief_avg_matchTime = mean(table2array(resultsFASTBRIEFsummary(2:end,8)));
brief_pct = brief_avg_matched/brief_avg_roi*100

freak_avg_roi = 	  mean(table2array(resultsFASTFREAKsummary(1:end,4)));
freak_avg_matched   = mean(table2array(resultsFASTFREAKsummary(2:end,5)));
freak_avg_desrTime  = mean(table2array(resultsFASTFREAKsummary(:,7)));
freak_avg_matchTime = mean(table2array(resultsFASTFREAKsummary(2:end,8)));
freak_pct = freak_avg_matched/freak_avg_roi*100

orb_avg_roi = 	    mean(table2array(resultsFASTORBsummary(1:end,4)));
orb_avg_matched   = mean(table2array(resultsFASTORBsummary(2:end,5)));
orb_avg_desrTime  = mean(table2array(resultsFASTORBsummary(:,7)));
orb_avg_matchTime = mean(table2array(resultsFASTORBsummary(2:end,8)));
orb_pct = orb_avg_matched/orb_avg_roi*100

sift_avg_roi = 	     mean(table2array(resultsFASTSIFTsummary(1:end,4)));
sift_avg_matched   = mean(table2array(resultsFASTSIFTsummary(2:end,5)));
sift_avg_desrTime  = mean(table2array(resultsFASTSIFTsummary(:,7)));
sift_avg_matchTime = mean(table2array(resultsFASTSIFTsummary(2:end,8)));
sift_pct = sift_avg_matched/sift_avg_roi*100



% ORB detector

brisk_avg_roi = 	  mean(table2array(resultsORBBRISKsummary(1:end,4)));
brisk_avg_matched   = mean(table2array(resultsORBBRISKsummary(2:end,5)));
brisk_avg_desrTime  = mean(table2array(resultsORBBRISKsummary(:,7)));
brisk_avg_matchTime = mean(table2array(resultsORBBRISKsummary(2:end,8)));
brisk_pct = brisk_avg_matched/brisk_avg_roi*100

brief_avg_roi = 	  mean(table2array(resultsORBBRIEFsummary(1:end,4)));
brief_avg_matched   = mean(table2array(resultsORBBRIEFsummary(2:end,5)));
brief_avg_desrTime  = mean(table2array(resultsORBBRIEFsummary(:,7)));
brief_avg_matchTime = mean(table2array(resultsORBBRIEFsummary(2:end,8)));
brief_pct = brief_avg_matched/brief_avg_roi*100

freak_avg_roi = 	  mean(table2array(resultsORBFREAKsummary(1:end,4)));
freak_avg_matched   = mean(table2array(resultsORBFREAKsummary(2:end,5)));
freak_avg_desrTime  = mean(table2array(resultsORBFREAKsummary(:,7)));
freak_avg_matchTime = mean(table2array(resultsORBFREAKsummary(2:end,8)));
freak_pct = freak_avg_matched/freak_avg_roi*100

orb_avg_roi = 	    mean(table2array(resultsORBORBsummary(1:end,4)));
orb_avg_matched   = mean(table2array(resultsORBORBsummary(2:end,5)));
orb_avg_desrTime  = mean(table2array(resultsORBORBsummary(:,7)));
orb_avg_matchTime = mean(table2array(resultsORBORBsummary(2:end,8)));
orb_pct = orb_avg_matched/orb_avg_roi*100

sift_avg_roi = 	     mean(table2array(resultsORBSIFTsummary(1:end,4)));
sift_avg_matched   = mean(table2array(resultsORBSIFTsummary(2:end,5)));
sift_avg_desrTime  = mean(table2array(resultsORBSIFTsummary(:,7)));
sift_avg_matchTime = mean(table2array(resultsORBSIFTsummary(2:end,8)));
sift_pct = sift_avg_matched/sift_avg_roi*100



% SIFT detector

brisk_avg_roi = 	  mean(table2array(resultsSIFTBRISKsummary(1:end,4)));
brisk_avg_matched   = mean(table2array(resultsSIFTBRISKsummary(2:end,5)));
brisk_avg_desrTime  = mean(table2array(resultsSIFTBRISKsummary(:,7)));
brisk_avg_matchTime = mean(table2array(resultsSIFTBRISKsummary(2:end,8)));
brisk_pct = brisk_avg_matched/brisk_avg_roi*100

brief_avg_roi = 	  mean(table2array(resultsSIFTBRIEFsummary(1:end,4)));
brief_avg_matched   = mean(table2array(resultsSIFTBRIEFsummary(2:end,5)));
brief_avg_desrTime  = mean(table2array(resultsSIFTBRIEFsummary(:,7)));
brief_avg_matchTime = mean(table2array(resultsSIFTBRIEFsummary(2:end,8)));
brief_pct = brief_avg_matched/brief_avg_roi*100

freak_avg_roi = 	  mean(table2array(resultsSIFTFREAKsummary(1:end,4)));
freak_avg_matched   = mean(table2array(resultsSIFTFREAKsummary(2:end,5)));
freak_avg_desrTime  = mean(table2array(resultsSIFTFREAKsummary(:,7)));
freak_avg_matchTime = mean(table2array(resultsSIFTFREAKsummary(2:end,8)));
freak_pct = freak_avg_matched/freak_avg_roi*100

% orb_avg_roi = 	    mean(table2array(resultsSIFTORBsummary(1:end,4)));
% orb_avg_matched   = mean(table2array(resultsSIFTORBsummary(2:end,5)));
% orb_avg_desrTime  = mean(table2array(resultsSIFTORBsummary(:,7)));
% orb_avg_matchTime = mean(table2array(resultsSIFTORBsummary(2:end,8)));
% orb_pct = orb_avg_matched/orb_avg_roi*100

sift_avg_roi = 	     mean(table2array(resultsSIFTSIFTsummary(1:end,4)));
sift_avg_matched   = mean(table2array(resultsSIFTSIFTsummary(2:end,5)));
sift_avg_desrTime  = mean(table2array(resultsSIFTSIFTsummary(:,7)));
sift_avg_matchTime = mean(table2array(resultsSIFTSIFTsummary(2:end,8)));
sift_pct = sift_avg_matched/sift_avg_roi*100
