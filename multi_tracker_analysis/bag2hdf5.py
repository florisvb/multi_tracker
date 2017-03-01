#!/usr/bin/env python

# written by sdvillal / StrawLab: https://github.com/strawlab/bag2hdf5/blob/master/bag2hdf5
# copied here for convenience

import os
import sys

import numpy as np
import argparse
import h5py

import roslib

roslib.load_manifest('rosbag')
import rosbag

import warnings
import progressbar

FLOAT_TIME = True  # False to prevent saving of float redundant timestamps


def flatten_msg(msg, t, max_strlen=None):
    assert max_strlen is not None  # don't accept default
    result = []
    for i, attr in enumerate(msg.__slots__):
        rostype = msg._slot_types[i]

        if attr == 'header':
            h = msg.header
            result.extend([h.seq,
                           h.stamp.secs,
                           h.stamp.nsecs,
                           h.frame_id,
                           ])
            if FLOAT_TIME:
                result.append(h.stamp.secs + h.stamp.nsecs*1e-9)
        
        elif rostype == 'time':
            p = getattr(msg, attr)
            result.extend([p.secs, p.nsecs])
            if FLOAT_TIME:
                result.append(p.secs + p.nsecs*1e-9)

        elif rostype == 'geometry_msgs/Point':
            p = getattr(msg, attr)
            result.extend([p.x, p.y, p.z])

        elif rostype == 'geometry_msgs/Quaternion':
            p = getattr(msg, attr)
            result.extend([p.x, p.y, p.z, p.w])
        
        elif rostype == 'std_msgs/MultiArrayLayout':
            pass
            
        elif '[]' in rostype and 'string' not in rostype:
            p = getattr(msg, attr)
            l = [i for i in p]
            result.extend(l)

        else:
            p = getattr(msg, attr)
            if rostype == 'string' or rostype == 'string[]':
                if rostype == 'string[]':
                    # List of strings gets joined to one string
                    warnings.warn('string array is joined to single string', RuntimeWarning, stacklevel=2)
                    p = ','.join(p)
                assert len(p) <= max_strlen
            result.append(p)
    # also do timestamp
    result.extend([t.secs, t.nsecs])
    if FLOAT_TIME:
        result.append(t.secs + t.nsecs * 1e-9)

    return tuple(result)


def rostype2dtype(rostype, max_strlen=None):
    assert max_strlen is not None  # don't accept default

    if rostype == 'float32':
        dtype = np.float32
    elif rostype == 'float64':
        dtype = np.float64
    elif rostype == 'uint8' or rostype == 'byte':
        dtype = np.uint8
    elif rostype == 'uint16':
        dtype = np.uint16
    elif rostype == 'uint32':
        dtype = np.uint32
    elif rostype == 'uint64':
        dtype = np.uint64
    elif rostype == 'int8':
        dtype = np.int8
    elif rostype == 'int16':
        dtype = np.int16
    elif rostype == 'int32':
        dtype = np.int32
    elif rostype == 'int64':
        dtype = np.int64
    elif rostype == 'bool' or rostype == 'bool[]':
        dtype = np.bool
    elif rostype == 'string' or rostype == 'string[]':
        dtype = 'S' + str(max_strlen)
    else:
        raise ValueError('unknown ROS type: %s' % rostype)
    return dtype


def make_dtype(msg, max_strlen=None):
    assert max_strlen is not None  # don't accept default

    result = []
    for i, attr in enumerate(msg.__slots__):
        rostype = msg._slot_types[i]
        
        if '[]' in rostype and 'string' not in rostype:
            p = getattr(msg, attr)
            length_of_msg = len(p)
        
        if rostype == 'Header' or rostype == 'std_msgs/Header':
            result.extend([('header_seq', np.uint32),
                           ('header_stamp_secs', np.int32),
                           ('header_stamp_nsecs', np.int32),
                           ('header_frame_id', 'S'+str(max_strlen))])
            if FLOAT_TIME:
                result.append(('header_stamp', np.float64))
        elif rostype == 'time':
            result.extend([('time_secs', np.int32),
                           ('time_nsecs', np.int32)])
            if FLOAT_TIME:
                result.append(('time', np.float64))
        elif rostype == 'geometry_msgs/Point':
            result.extend([(attr+'_x', np.float32),
                           (attr+'_y', np.float32),
                           (attr+'_z', np.float32),
                           ])
        elif rostype == 'geometry_msgs/Quaternion':
            result.extend([(attr+'_x', np.float32),
                           (attr+'_y', np.float32),
                           (attr+'_z', np.float32),
                           (attr+'_w', np.float32),
                           ])
        elif rostype == 'std_msgs/MultiArrayLayout':
            pass
        elif '[]' in rostype and 'string' not in rostype:
            basetype = rostype.split('[]')[0]
            r = []
            for i in range(length_of_msg):
                r.append( (attr+'_'+str(i), np.__getattribute__(basetype)) )
            result.extend(r)

        else:
            nptype = rostype2dtype(rostype, max_strlen=max_strlen)
            result.append((attr, nptype))
    # also do timestamp
    result.extend([('t_secs', np.int32), ('t_nsecs', np.int32)])
    if FLOAT_TIME:
        result.append(('t', np.float64))
    return result


def h5append(dset, arr):
    n_old_rows = dset.shape[0]
    n_new_rows = len(arr) + n_old_rows
    dset.resize(n_new_rows, axis=0)
    dset[n_old_rows:] = arr


def bag2hdf5(fname, out_fname, topics=None, max_strlen=None, skip_messages={}):
    assert max_strlen is not None  # don't accept default

    bag = rosbag.Bag(fname)
    results2 = {}
    chunksize = 10000
    dsets = {}
    
    # progressbar
    _pbw = ['converting %s: ' % fname, progressbar.Percentage()]
    pbar = progressbar.ProgressBar(widgets=_pbw, maxval=bag.size).start()
    
    if topics is None:
        print 'AUTO FIND TOPICS'
        topics = []
        for topic, msg, t in bag.read_messages():
            topics.append(topic)
        topics = np.unique(topics).tolist()
        print topics
        
    print 'skip messages: '
    print skip_messages
    print
    
    try:
        with h5py.File(out_fname, mode='w') as out_f:
            for topic in topics:
                m = -1
                for topic, msg, t in bag.read_messages(topics=[topic]):
                    m += 1
                    
                    
                    if topic not in skip_messages.keys():
                        skip_messages[topic] = []
                    
                    #print topic, m, skip_messages[topic]
                    
                    # update progressbar
                    pbar.update(bag._file.tell())
                    # get the data
                    
                    if m not in skip_messages[topic]:
                        this_row = flatten_msg(msg, t, max_strlen=max_strlen)
                        
                        # convert it to numpy element (and dtype)
                        if topic not in results2:
                            try:
                                dtype = make_dtype(msg, max_strlen=max_strlen)
                            except:
                                print >> sys.stderr, "*********************************"
                                print >> sys.stderr, 'topic:', topic
                                print >> sys.stderr, "\nerror while processing message:\n\n%r" % msg
                                print >> sys.stderr, '\nROW:', this_row
                                print >> sys.stderr, "*********************************"
                                raise
                            results2[topic] = dict(dtype=dtype,
                                                   object=[this_row])
                        else:
                            results2[topic]['object'].append(this_row)
                    
                        
                        # now flush our caches periodically
                        if len(results2[topic]['object']) >= chunksize:
                            arr = np.array(**results2[topic])
                            if topic not in dsets:
                                # initial creation
                                dset = out_f.create_dataset(topic, data=arr, maxshape=(None,),
                                                            compression='gzip',
                                                            compression_opts=9)
                                assert dset.compression == 'gzip'
                                assert dset.compression_opts == 9
                                dsets[topic] = dset
                            else:
                                # append to existing dataset
                                h5append(dsets[topic], arr)
                            del arr
                            # clear the cached values
                            results2[topic]['object'] = []
                    
                    else:
                        print 'skipping message: ', m
            # done reading bag file. flush remaining data to h5 file
            for topic in results2:
                print topic
                print results2[topic]
                print
                if not len(results2[topic]['object']):
                    # no data
                    continue
                arr = np.array(**results2[topic])
                if topic in dsets:
                    h5append(dsets[topic], arr)
                else:
                    out_f.create_dataset(topic,
                                         data=arr,
                                         compression='gzip',
                                         compression_opts=9)

    except:
        if os.path.exists(out_fname):
            os.unlink(out_fname)
        raise
    finally:
        pass
        pbar.finish()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', type=str, help="the .bag file")
    parser.add_argument('--max_strlen', type=int, default=255,
                        help="maximum length of encoded strings")
    parser.add_argument('--out', type=str, default=None,
                        help="name of output file")
    parser.add_argument('--topic', type=str, nargs='*',
                        help="topic name to convert. defaults to all. "
                        "multiple may be specified.")
    args = parser.parse_args()

    if not os.path.exists(args.filename):
        print >> sys.stderr, 'No file %s' % args.filename
        sys.exit(1)
    fname = os.path.splitext(args.filename)[0]
    if args.out is not None:
        output_fname = args.out
    else:
        output_fname = fname + '.hdf5'
        if os.path.exists(output_fname):
            print >> sys.stderr, 'will not overwrite %s' % output_fname
            sys.exit(1)

    bag2hdf5(args.filename,
             output_fname,
             max_strlen=args.max_strlen,
             topics=args.topic)
