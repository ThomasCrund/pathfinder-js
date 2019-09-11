{
  "targets": [
   {
      'include_dirs': [
        '..',
        'include/'
      ],
      "target_name": "pathfinder",
      "sources": [  "main.cc",
                    #"test.cc",
                    'src/mathutil.c',
                    'src/io.c',
                    'src/spline.c',
                    'src/trajectory.c',
                    'src/generator.c',
                    'src/error.c',
                    'src/fit/hermite.c',
                    'src/modifiers/tank.c',
                    'src/modifiers/swerve.c',          
      ],
      'cflags!': [ '-fno-exceptions' ],
      'cflags_cc!': [ '-fno-exceptions' ],
      'conditions': [
        ['OS=="mac"', {
          'xcode_settings': {
            'GCC_ENABLE_CPP_EXCEPTIONS': 'YES'
          }
        }]
      ]
    }
  ]
}
