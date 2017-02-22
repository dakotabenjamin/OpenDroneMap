import ecto

from opendm import io
from opendm import log
from opendm import system
from opendm import context
from opendm import types


class ODMOrthoPhotoCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("resolution", 'Orthophoto ground resolution in pixels/meter', 20)
        params.declare("verbose", 'print additional messages to console', False)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):

        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running ODM Orthophoto Cell')

        # get inputs
        args = self.inputs.args
        tree = self.inputs.tree
        verbose = '-verbose' if self.params.verbose else ''

        # define paths and create working directories
        system.mkdir_p(tree.odm_orthophoto)

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'odm_orthophoto') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'odm_orthophoto' in args.rerun_from)

        if not io.file_exists(tree.odm_orthophoto_file) or rerun_cell:

            # odm_orthophoto definitions
            kwargs = {
                'bin': context.odm_modules_path,
                'log': tree.odm_orthophoto_log,
                'ortho': tree.odm_orthophoto_file,
                'corners': tree.odm_orthophoto_corners,
                'res': self.params.resolution,
                'verbose': verbose
            }

            kwargs['model_geo'] = tree.odm_georeferencing_model_obj_geo \
                if io.file_exists(tree.odm_georeferencing_coords) \
                else tree.odm_textured_model_obj


            # run odm_orthophoto
            system.run('{bin}/odm_orthophoto -inputFile {model_geo} '
                       '-logFile {log} -outputFile {ortho} -resolution {res} {verbose} '
                       '-outputCornerFile {corners}'.format(**kwargs))

            if not io.file_exists(tree.odm_georeferencing_coords):
                log.ODM_WARNING('No coordinates file. A georeferenced raster '
                                'will not be created')
            else:
                # Create georeferenced GeoTiff
                geotiffcreated = False
                georef = types.ODM_GeoRef()
                # creates the coord refs # TODO I don't want to have to do this twice- after odm_georef
                georef.parse_coordinate_system(tree.odm_georeferencing_coords)

                if georef.epsg and georef.utm_east_offset and georef.utm_north_offset:
                    ulx = uly = lrx = lry = 0.0
                    with open(tree.odm_orthophoto_corners) as f:
                        for lineNumber, line in enumerate(f):
                            if lineNumber == 0:
                                tokens = line.split(' ')
                                if len(tokens) == 4:
                                    ulx = float(tokens[0]) + \
                                        float(georef.utm_east_offset)
                                    lry = float(tokens[1]) + \
                                        float(georef.utm_north_offset)
                                    lrx = float(tokens[2]) + \
                                        float(georef.utm_east_offset)
                                    uly = float(tokens[3]) + \
                                        float(georef.utm_north_offset)
                    log.ODM_INFO('Creating GeoTIFF')

                    kwargs = {
                        'ulx': ulx,
                        'uly': uly,
                        'lrx': lrx,
                        'lry': lry,
                        'epsg': georef.epsg,
                        'png': tree.odm_orthophoto_file,
                        'tiff': tree.odm_orthophoto_tif,
                        'log': tree.odm_orthophoto_tif_log
                    }

                    system.run('gdal_translate -a_ullr {ulx} {uly} {lrx} {lry} '
                                '-co TILED=yes '
                                '-co COMPRESS=DEFLATE '
                                '-co PREDICTOR=2 '
                                '-co BLOCKXSIZE=512 '
                                '-co BLOCKYSIZE=512 '
                                '-co NUM_THREADS=ALL_CPUS '
                               '-a_srs \"EPSG:{epsg}\" {png} {tiff} > {log}'.format(**kwargs))
                    geotiffcreated = True
                if not geotiffcreated:
                    log.ODM_WARNING('No geo-referenced orthophoto created due '
                                    'to missing geo-referencing or corner coordinates.')
                if args.ndvi:
                    import numpy
                    # Take the orthophoto and do nir - vis / nir + vis
                    # Temporary hardcoded bands
                    nirband = 1
                    visband = 2
                    # import raster
                    raster = gdal.Open(tree.odm_orthophoto_tif)
                    rarray = raster.ReadAsArray()

                    # Export info
                    outfile = 'odm_ndvi.tif' # TODO: Put in file tree

                    # parse out bands
                    nirb = rarray[nirband - 1]
                    visb = rarray[visband - 1]

                    visb = visb.astype(float)
                    nirb = nirb.astype(float)

                    # nirb = numpy.ma.array(nirb, mask = ( nirb == 0 ))
                    # visb = numpy.ma.array(visb, mask = ( nirb == 0 ))

                    # for each cell, calculate ndvi (masking out where divide by 0)
                    mask = numpy.not_equal((nirb + visb), 0)
                    ndvi = numpy.choose(mask, (-99, (nirb - visb) / (nirb + visb)))

                    # export raster
                    # get resolutions
                    out_driver = gdal.GetDriverByName('GTiff').Create(outfile, int(ndvi.shape[1]), int(ndvi.shape[0]), 1)
                    outband = out_driver.GetRasterBand(1)
                    outband.WriteArray(ndvi)



        else:
            log.ODM_WARNING('Found a valid orthophoto in: %s' % tree.odm_orthophoto_file)

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'Orthophoto')

        log.ODM_INFO('Running ODM OrthoPhoto Cell - Finished')
        return ecto.OK if args.end_with != 'odm_orthophoto' else ecto.QUIT
