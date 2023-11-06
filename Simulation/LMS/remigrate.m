function remigrate()
%REMIGRATE restore original model from migration backup and then migrate again

%restore old model
backupfolder = '.\Migrated\version_[]_155222' ;
backupmodel  = 'LMS_[]_155222_cs' ;
backupfile   = '.\Migrated\version_[]_155222\LMS_[]_155222_cs.slx' ;
srcfolder    = '.' ;
srcmodel     = 'LMS_cs' ;
modelreferencing = 0 ;
mbxutils.backupModel(backupfolder,backupmodel,srcfolder,srcmodel); 
chdir('.');
open_system('LMS_cs');

%migrate restored model
migrate_all(srcmodel,modelreferencing,backupfile);
end
