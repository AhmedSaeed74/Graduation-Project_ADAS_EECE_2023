function unmigrate()
%UNMIGRATE restore original model from migration backup
backupfolder = '.\Migrated\version_[]_155222' ;
backupmodel  = 'LMS_[]_155222_cs' ;
srcfolder    = '.' ;
srcmodel     = 'LMS_cs' ;
mbxutils.backupModel(backupfolder,backupmodel,srcfolder,srcmodel); 
chdir('.');
open_system('LMS_cs');
end
