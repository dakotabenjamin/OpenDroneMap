import ecto, json, os

from opendm import log
from opendm import io
from opendm import system
from opendm import context


class ODMeshingCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("max_vertex", 'The maximum vertex count of the output '
                                     'mesh', 100000)
        params.declare("oct_tree", 'Oct-tree depth used in the mesh reconstruction, '
                                   'increase to get more vertices, recommended '
                                   'values are 8-12', 9)
        params.declare("samples", 'Number of points per octree node, recommended '
                                  'value: 1.0', 1)
        params.declare("solver", 'Oct-tree depth at which the Laplacian equation '
                                 'is solved in the surface reconstruction step. '
                                 'Increasing this value increases computation '
                                 'times slightly but helps reduce memory usage.', 9)

        params.declare("wlop_iterations", 'Iterations of the Weighted Locally Optimal Projection (WLOP) simplification algorithm. '
                                          'Higher values take longer but produce a smoother mesh. '
                                          'Applies to 2.5D mesh only. ', 70)

        params.declare("verbose", 'print additional messages to console', False)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction", "Clusters output. list of ODMReconstructions", [])
        outputs.declare("reconstruction", "Clusters output. list of ODMReconstructions", [])

    def process(self, inputs, outputs):

        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running ODM Meshing Cell')

        # get inputs
        args = self.inputs.args
        tree = self.inputs.tree
        verbose = '-verbose' if self.params.verbose else ''

        # define paths and create working directories
        system.mkdir_p(tree.odm_meshing)

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'odm_meshing') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'odm_meshing' in args.rerun_from)

        infile = tree.opensfm_model
        if args.use_pmvs:
            infile = tree.pmvs_model
        if args.sparse:
            infile = tree.opensfm_sparse_model

        if not io.file_exists(tree.odm_mesh) or rerun_cell:
            log.ODM_DEBUG('Writing ODM Mesh file in: %s' % tree.odm_mesh)

            kwargs = {
                'bin': context.odm_modules_path,
                'outfile': tree.odm_mesh,
                'infile': infile,
                'log': tree.odm_meshing_log,
                'max_vertex': self.params.max_vertex,
                'oct_tree': self.params.oct_tree,
                'samples': self.params.samples,
                'solver': self.params.solver,
                'verbose': verbose
            }

            # run meshing binary
            system.run('{bin}/odm_meshing -inputFile {infile} '
                       '-outputFile {outfile} -logFile {log} '
                       '-maxVertexCount {max_vertex} -octreeDepth {oct_tree} {verbose} '
                       '-samplesPerNode {samples} -solverDivide {solver}'.format(**kwargs))
        else:
            log.ODM_WARNING('Found a valid ODM Mesh file in: %s' %
                            tree.odm_mesh)

        # Do we need to generate a 2.5D mesh also?
        if args.use_25dmesh:
          if not io.file_exists(tree.odm_25dmesh) or rerun_cell:
              log.ODM_DEBUG('Writing ODM 2.5D Mesh file in: %s' % tree.odm_25dmesh)

              upside_down = self.is_point_cloud_upside_down(infile)
              log.ODM_INFO('%s is upside down: ' % 'YES' if upside_down else 'NO')

              # Use PDAL to:
              # 1. Do statistical outliers removal
              # 2. Split ground vs non-ground points
              #
              # If the point cloud is flipped upside down, we need to 
              # flip it via transformation prior to applying the smrf filter
              # otherwise we'll get whacky results.
              classified_output_file = os.path.join(tree.odm_meshing, 'filtered_ground_vs_nonground.ply')
              json_file = os.path.join(tree.odm_meshing, 'filtered_ground_vs_nonground.json')

              pipeline = {'pipeline': 
              [
                {
                  "type":"readers.ply",
                  "filename":infile
                },
                {
                  "type":"filters.outlier",
                  "method":"statistical"
                },
                {
                  "type":"filters.range",
                  "limits":"Classification![7:7]"
                }
              ]}
              
              steps = pipeline['pipeline']
              rotate_180_x_axis = "1 0 0 0 0 -1 0 0 0 0 -1 0 0 0 0 1"

              if upside_down:
                steps.append({
                    "type":"filters.transformation",
                    "matrix": rotate_180_x_axis
                  })

              steps.append({
                  "type": "filters.smrf"
                })

              if upside_down:
                steps.append({
                    "type":"filters.transformation",
                    "matrix": rotate_180_x_axis
                  })
              steps.append({
                "type":"writers.ply",
                "filename":classified_output_file
              })

              # Execute pipeline
              with open(json_file, 'w') as f:
                f.write(json.dumps(pipeline))

              system.run('{}/pdal pipeline -i {}'.format(context.pdal_path, json_file))

              # Run 2.5D meshing binary
              kwargs = {
                  'bin': context.odm_modules_path,
                  'outfile': tree.odm_25dmesh,
                  'infile': classified_output_file,
                  'log': tree.odm_25dmeshing_log,
                  'verbose': verbose,
                  'max_vertex': self.params.max_vertex,
                  'wlop_iterations': self.params.wlop_iterations
              }

              system.run('{bin}/odm_25dmeshing -inputFile {infile} '
                         '-outputFile {outfile} -logFile {log} '
                         '-maxVertexCount {max_vertex} '
                         '-wlopIterations {wlop_iterations} {verbose}'.format(**kwargs))
          else:
              log.ODM_WARNING('Found a valid ODM 2.5D Mesh file in: %s' %
                              tree.odm_25dmesh)

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'Meshing')

        log.ODM_INFO('Running ODM Meshing Cell - Finished')
        return ecto.OK if args.end_with != 'odm_meshing' else ecto.QUIT


    def is_point_cloud_upside_down(self, ply_path):
      try:
        with open(ply_path, 'r') as f:
          if f.readline().rstrip() != 'ply': raise ValueError('Invalid PLY file')
          if f.readline().rstrip() != 'format ascii 1.0': raise ValueError('Non-ascii ply file (not supported)')

          has_normals = False

          while True:
            line = f.readline()
            if not line: break # EOF
            if line.rstrip() == 'property float nz': has_normals = True
            if line.rstrip() == 'end_header': break

          if not has_normals: raise ValueError('PLY file does not have normals')

          zNormalsDirectionCount = 0

          for line in f:
            nz = float(line.split(' ')[5])
            zNormalsDirectionCount += 1 if nz >= 0 else -1

          return zNormalsDirectionCount < 0
      except (ValueError, IOError) as e:
        log.ODM_WARNING('Could not read {}: {}, will assume point cloud is aligned in the up direction.'.format(ply_path, e.message))
        return False


