(in-package "COMPILER")
(defmethod translator
 (:enter (cname lname)
    (terpri cfile)
    (send self :comment lname)
    (setq pushcount 0)          ;reset pushcount
                                ;because a4(vfp) changes here
    (format cfile "static pointer ~A(ctx,n,argv,env)~%" cname)
    (format cfile "register context *ctx;~%")
    (format cfile "register int n; register pointer argv[]; pointer env;~%")
    (format cfile "{ register pointer *local=ctx->vsp;~%")
    (format cfile "  volatile pointer w;~%")
    (format cfile "  register pointer *fqv=qv;~%")
    (format cfile "  volatile numunion nu;~%")
    )
 )
