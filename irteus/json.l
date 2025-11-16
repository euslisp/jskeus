(defun parse-json-file (file-path)
  "Euslisp simple json loader"
  (with-open-file (strm file-path)
    (parse-json-stream strm)))

(defun access-alist (alist key-str)
  (cdr (assoc key-str alist :test 'string-equal)))

(defun char2str (c)
  (concatenate string "" (vector c)))

(defun str2vector (str)
  (concatenate vector "" str))

(defun parse-json-stream (strm)
  (skip-whitespace strm)
  (let ((c (peek-char strm)))
    (cond ((= c #\{) (parse-json-object strm))
          ((= c #\[) (parse-json-array strm))
          ((= c #\") (parse-json-string strm))
          ((or (= c #\-) (digit-char-p c)) (parse-json-number strm))
          ((alpha-char-p c) (parse-json-constant strm))
          (t (error "Unexpected character in JSON: ~A" c)))))

(defun parse-json-object (strm)
  (read-char strm) ;; Consume '{'
  (let ((obj '()))
    (loop
      (skip-whitespace strm)
      (when (= (peek-char strm) #\}) (read-char strm) (return obj))
      (let ((key (parse-json-string strm)))
        (skip-whitespace strm)
        (unless (= (read-char strm) #\:) (error "Expected ':' after key"))
        (let ((value (parse-json-stream strm)))
          ;;(push (cons key value) obj)
          (setq obj (acons key value obj))
          ))
      (skip-whitespace strm)
      (when (= (peek-char strm) #\,) (read-char strm))
      (skip-whitespace strm))
    (nreverse obj))
  )

(defun parse-json-array (strm)
  (read-char strm) ;; Consume '['
  (let ((arr (vector)))
    (loop
      (skip-whitespace strm)
      (when (= (peek-char strm) #\]) (read-char strm) (return (nreverse arr)))
      (setq arr (concatenate vector (vector (parse-json-stream strm)) arr))
      (skip-whitespace strm)
      (when (= (peek-char strm) #\,) (read-char strm))
      (skip-whitespace strm))))

(defun parse-json-string (strm)
  (read-char strm) ;; Consume '"'
  (let ((str ""))
    (loop
      (let* ((c (read-char strm)))
        (cond ((= c #\") (return str))
              ((= c #\\) ;; Escape sequences
               (let ((next (read-char strm)))
                 (setq str (concatenate string str
                                               (char2str
                                                 (case next
                                                          (#\" #\")
                                                          (#\\ #\\)
                                                          (#\/ #\/)
                                                          (#\b #\backspace)
                                                          (#\f #\page)
                                                          (#\n #\newline)
                                                          (#\r #\return)
                                                          (#\t #\tab)
                                                          (t (error "Invalid escape sequence: ~A" next))))))))
              (t 
                (progn
                  (setq str (concatenate string str (char2str c)))
                  )
                ))
        ))))

(defun parse-json-number (strm)
  (let ((num-str "") ans)
    (loop
      (let ((c (peek-char strm)))
        (if (or (digit-char-p c) (= c #\-) (= c #\.))
            (progn
              (setq num-str (concatenate string num-str (char2str (read-char strm))))
              )
            (progn
              (read (make-string-input-stream "1"))
              (read-from-string num-str)
              (setq ans (read-from-string num-str))
              (return ans)
              )
            )))))

(defun parse-json-constant (strm)
  (let ((const-str ""))
    (loop
      (let ((c (peek-char strm)))
        (if (alpha-char-p c)
            (progn
              (setq const-str (concatenate string const-str (char2str (read-char strm)))))
            (return (cond ((string= const-str "true") t)
                          ((string= const-str "false") nil)
                          ((string= const-str "null") nil)
                          (t (error "Unknown constant: ~A" const-str)))))))))

(defun skip-whitespace (strm)
  (loop
    (let ((c (peek-char strm)))
      (if (or (= c #\space) (= c #\newline) (= c #\tab))
          (read-char strm)
          (return)))))

