#!/usr/bin/env python3
import os

'''Delete Amazon Cookies on Firefox'''
os.system('sqlite3 ~/.mozilla/firefox/*.default-release/cookies.sqlite \\ \'delete from moz_cookies where host LIKE "%amazon.%";\'')
os.system('sqlite3 ~/.mozilla/firefox/*.default/cookies.sqlite \\ \'delete from moz_cookies where host LIKE "%amazon.%";\'')
os.system('sqlite3 ~/.mozilla/firefox/*.default-release/cookies.sqlite \\ \'delete from moz_cookies where host LIKE "%www.amazon.%";\'')
os.system('sqlite3 ~/.mozilla/firefox/*.default/cookies.sqlite \\ \'delete from moz_cookies where host LIKE "%www.amazon.%";\'')

'''Delete Amazon Cookies on Chrome'''
os.system('sqlite3 ~/.config/google-chrome/Default/Cookies \\ \'delete from cookies where host_key LIKE "%amazon.%";\'')

print("Cookies Deleted")