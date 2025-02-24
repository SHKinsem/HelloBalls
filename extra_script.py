import os

def before_upload(source, target, env):
    print("Stopping helloballs.service...")
    os.system("sudo systemctl stop helloballs.service")
    print("helloballs.service stopped successfully!")

def after_upload(source, target, env):
    print("Restarting helloballs.service...")
    os.system("sudo systemctl restart helloballs.service")
    print("helloballs.service restarted successfully!")

Import("env")
env.AddPreAction("upload", before_upload)
env.AddPostAction("upload", after_upload)
