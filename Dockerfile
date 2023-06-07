FROM russtedrake/manipulation:cfccef8

# add python packages in requirements.txt
ADD requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

# add the rest of the source code
WORKDIR /code

# run the jupyter notebook
CMD ["jupyter", "notebook", "--ip=0.0.0.0","--port", "8080", "--allow-root", "--no-browser"]