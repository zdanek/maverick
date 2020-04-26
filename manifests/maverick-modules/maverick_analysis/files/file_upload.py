#!/usr/bin/env python3
# Very simple file upload script, hacked on code taken from https://technobeans.com/2012/09/17/tornado-file-uploads/

import tornado
import tornado.ioloop
import tornado.web
import sqlite3
import os, uuid

# Define paths
baseDir = "/srv/maverick/data/analysis/"
inbox = os.path.join(baseDir, "inbox/")
anonybox = os.path.join(baseDir, "anonybox/")

class Userform(tornado.web.RequestHandler):
    def get(self):
        self.render("index.html", fname=None, _uploadDir=None)

class Upload(tornado.web.RequestHandler):
    def sanitise_file(self, filename):
        filename = filename.replace(" ", "_")
        return filename
        
    def post(self):
        try:
            fileinfo = self.request.files['file'][0]
            filename = self.sanitise_file(fileinfo['filename'])
        except:
            # If no filename, skip and render upload form
            self.render("index.html", fname=None, _uploadDir=None)
            return

        anon = self.get_argument("anon", None, False)
        if anon:
            _uploadDir = anonybox
        else:
            _uploadDir = inbox

        # Write an entry to the meta db
        description = self.get_argument("description", None, False)
        try:
            conn = sqlite3.connect("/srv/maverick/data/analysis/mavlog-meta.db")
            cursor = conn.cursor()
            # See if entry already exists
            cursor.execute("SELECT id FROM logfiles WHERE filename='"+filename+"'")
            try:
                id_exists = cursor.fetchone()[0]
            except:
                id_exists = None
            if not id_exists:
                cursor.execute('INSERT INTO logfiles (filename,state,description) VALUES (?,?,?)', (filename,"uploaded",description))
            else:
                cursor.execute('UPDATE logfiles SET start=?,state=?,description=? WHERE id=?', (filename,"uploaded",description,id_exists))
            conn.commit()
            conn.close()
        except Exception as e:
            print("Meta entry exception: {}".format(repr(e)))

        # Write the upload contents to a new file in inbox/anonybox
        fh = open(os.path.join(_uploadDir, filename), 'w')
        fh.write(fileinfo['body'])
        self.render("index.html", fname=filename, _uploadDir=_uploadDir)

application = tornado.web.Application([
        (r"/", Userform),
        (r"/upload", Upload),
        ], debug=True)


if __name__ == "__main__":
    application.listen(6792)
    tornado.ioloop.IOLoop.instance().start()